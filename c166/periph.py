"""C167 peripherals, as boards built on this core use them: the general
purpose timers, the serial port (a K line), the synchronous serial port,
the on-chip CAN module, the A/D converter, and an Am29F400BB's command
interface. They started in gs20-emu's board.py.

A device is given the board it sits on, and uses of it: `cpu`, `ips`
(instructions per second of the module's time), `fcpu` (the CPU clock),
`plan()` (recompute the CPU's next event from the devices' `next_at`),
`unsupported` (a set for configurations that are not modelled) and
`read16`. Each device has `next_at`, the instruction count at which it next
has something to do, and `service(now)`.
"""
from . import sfr
from .cpu import NEVER


class Timer:
    """One of the general purpose timers T2-T6, in timer mode: counting
    the CPU clock through a prescaler, up or down, with an interrupt
    request when it wraps. The other modes (counter, gated, capture,
    reload) leave the timer standing and are noted in board.unsupported."""

    RUN, DOWN = 0x40, 0x80

    def __init__(self, board, name, base_step, reload=None):
        self.b = board
        self.name = name
        self.value_at, self.con_at, self.ic = getattr(sfr, name), getattr(sfr, name + "CON"), getattr(sfr, name + "IC")
        self.base_step = base_step * board.ips // board.fcpu      # instructions per count with the prescaler at 0
        assert self.base_step >= 1, f"{name} cannot count at {board.ips} instructions a second"
        base_step = self.base_step
        self.reload = reload                     # (control bit, register): T6 reloads from CAPREL
        self.value, self.stamp, self.step, self.running, self.down = 0, 0, base_step, False, False
        self.next_at = NEVER
        self.wraps = 0
        # called at each overflow/underflow: T6's clocks the CAPCOM timers in
        # counter mode with input x00 (CapcomTimers.t6_wrapped)
        self.on_wrap = []
        cpu = board.cpu
        cpu.hook_read(self.value_at, lambda a: self.now())
        cpu.hook_write(self.value_at, self._set_value)
        cpu.hook_write(self.con_at, self._set_control)

    def now(self):
        if not self.running:
            return self.value
        ticks = (self.b.cpu.instructions - self.stamp) // self.step
        return (self.value - ticks if self.down else self.value + ticks) & 0xFFFF

    def _plan(self):
        self.next_at = self.stamp + ((self.value + 1) if self.down else (0x10000 - self.value)) * self.step if self.running else NEVER
        self.b.plan()

    def _set_value(self, addr, value):
        self.value, self.stamp = value, self.b.cpu.instructions
        self._plan()
        return value

    def _set_control(self, addr, con):
        self.value, self.stamp = self.now(), self.b.cpu.instructions
        mode = (con >> 3) & 7
        self.running = bool(con & self.RUN) and mode == 0
        self.down = bool(con & self.DOWN)
        self.step = self.base_step << (con & 7)
        if con & self.RUN and mode:
            self.b.unsupported.add(f"{self.name} mode {mode}")
        self._plan()
        return con

    def service(self, now):
        if now < self.next_at:
            return
        self.wraps += 1
        self.stamp = self.next_at
        self.value = 0xFFFF if self.down else 0
        if self.reload and self.b.cpu.rw(self.con_at) & self.reload[0]:
            self.value = self.b.cpu.rw(self.reload[1])
        self.b.cpu.request(self.ic)
        self._plan()
        for fn in self.on_wrap:
            fn()


class Capin:
    """GPT2's capture input CAPIN (P3.2): on the edge T5CON's CI field
    selects (01 rising, 10 falling, 11 either), with T5SC set, CAPREL takes
    T5's value, T5 is cleared if T5CLR is set, and the CAPREL interrupt
    (CRIC) is requested. Whoever drives the pin calls `edge(level)`; `t5`
    is the board's T5 Timer."""

    SC, CLR = 0x8000, 0x4000

    def __init__(self, board, t5):
        self.b = board
        self.t5 = t5
        self.level = 1
        self.captures = 0

    def edge(self, level):
        level = 1 if level else 0
        was, self.level = self.level, level
        if was == level:
            return
        cpu = self.b.cpu
        con = cpu.rw(sfr.T5CON)
        ci = (con >> 12) & 3
        if not con & self.SC or not ci or (ci == 1 and not level) or (ci == 2 and level):
            return
        value = self.t5.now()
        cpu.low[sfr.CAPREL], cpu.low[sfr.CAPREL + 1] = value & 0xFF, value >> 8
        if con & self.CLR:
            self.t5._set_value(sfr.T5, 0)
        self.captures += 1
        cpu.request(sfr.CRIC)


class Asc0:
    """The serial port, which is the K line. A byte written to S0TBUF is
    on the line one frame later (`tx`, `on_tx`), with the transmit
    interrupts the firmware's driver runs on, and is heard by the receiver
    as well: the K line is one wire. Bytes from a tester go in with
    `receive()` and arrive one per frame time at the programmed rate."""

    RUN, RECEIVE, BRS = 0x8000, 0x0010, 0x2000
    FRAME_BITS = {0: 10, 1: 10, 2: 9, 3: 10, 4: 11, 5: 11, 6: 10, 7: 11}      # by S0M: start, data, parity, stop

    def __init__(self, board, echo=True):
        self.b = board
        self.echo = echo
        self.tx = bytearray()
        self.on_tx = None
        self.shifting = None                     # the byte on its way out
        self.waiting = None                      # the one in S0TBUF behind it
        self.tx_at = NEVER
        self.rx_queue = []
        self.rx_at = NEVER
        self.rbuf = 0
        self.overruns = 0
        self.level = 1                           # the line as the RxD pin (P3.11) sees it: idle is high
        self.next_at = NEVER
        cpu = board.cpu
        cpu.hook_write(sfr.S0TBUF, self._write)
        cpu.hook_read(sfr.S0RBUF, lambda a: self.rbuf)
        # The firmware also looks at the pin itself: a line that is low is
        # busy (or a 5-baud address on its way), and it will not take a
        # telegram until the line has been high for a while.
        cpu.hook_read(sfr.P3, lambda a: (cpu.low[sfr.P3] | (cpu.low[sfr.P3 + 1] << 8)) & ~0x0800 | (self.level << 11))

    def frame(self):
        """Instructions one character takes at the programmed rate."""
        cpu = self.b.cpu
        con = cpu.rw(sfr.S0CON)
        clocks = self.FRAME_BITS[con & 7] * 16 * (3 if con & self.BRS else 2) * ((cpu.rw(sfr.S0BG) & 0x1FFF) + 1)
        return max(clocks * self.b.ips // self.b.fcpu, 1)

    def baud(self):
        cpu = self.b.cpu
        return self.b.fcpu / (16 * (3 if cpu.rw(sfr.S0CON) & self.BRS else 2) * ((cpu.rw(sfr.S0BG) & 0x1FFF) + 1))

    def _plan(self):
        self.next_at = min(self.tx_at, self.rx_at)
        self.b.plan()

    def _write(self, addr, value):
        if not self.b.cpu.rw(sfr.S0CON) & self.RUN:
            return value
        if self.shifting is None:
            self.shifting = value & 0xFF
            self.tx_at = self.b.cpu.instructions + self.frame()
            self.b.cpu.request(sfr.S0TBIC)       # the buffer is free again at once
            self._plan()
        else:
            self.waiting = value & 0xFF
        return value

    def receive(self, data):
        """Bytes from the tester."""
        if not self.rx_queue:
            self.rx_at = self.b.cpu.instructions + self.frame()
        self.rx_queue.extend(bytes(data))
        self._plan()

    def _deliver(self, byte):
        cpu = self.b.cpu
        if not cpu.rw(sfr.S0CON) & self.RECEIVE:
            return
        if cpu.low[sfr.S0RIC] & 0x80:
            self.overruns += 1                   # the last one was never fetched
        self.rbuf = byte
        cpu.request(sfr.S0RIC)

    def service(self, now):
        if now >= self.tx_at:
            byte, self.shifting = self.shifting, None
            self.tx.append(byte)
            if self.on_tx is not None:
                self.on_tx(byte)
            self.b.cpu.request(sfr.S0TIC)
            if self.echo:
                self._deliver(byte)
            if self.waiting is not None:
                self.shifting, self.waiting = self.waiting, None
                self.tx_at = now + self.frame()
                self.b.cpu.request(sfr.S0TBIC)
            else:
                self.tx_at = NEVER
        if now >= self.rx_at:
            self._deliver(self.rx_queue.pop(0))
            self.rx_at = now + self.frame() if self.rx_queue else NEVER
        self._plan()

    @property
    def idle(self):
        return self.shifting is None and not self.rx_queue


class Ssc:
    """The synchronous serial port, as a master: the SPI link to the
    output driver chip. A word written to SSCTB is shifted out and what
    the device sent back is in SSCRB a word time later, with the transmit
    and receive interrupt requests. `device(word)` plays the other end;
    nobody answering reads as zeros."""

    def __init__(self, board):
        self.b = board
        self.device = lambda word: 0
        self.sent = []
        self.out = None
        self.next_at = NEVER
        board.cpu.hook_write(sfr.SSCTB, self._write)

    def _write(self, addr, value):
        cpu = self.b.cpu
        self.out = value
        self.next_at = cpu.instructions + max(32 * ((cpu.rw(sfr.SSCBR) & 0xFFFF) + 1) * self.b.ips // self.b.fcpu, 1)
        self.b.plan()
        return value

    def service(self, now):
        if now < self.next_at:
            return
        cpu = self.b.cpu
        self.next_at = NEVER
        if len(self.sent) < 1000:
            self.sent.append(self.out)
        back = self.device(self.out) & 0xFFFF
        cpu.low[sfr.SSCRB], cpu.low[sfr.SSCRB + 1] = back & 0xFF, back >> 8
        cpu.request(sfr.SSCTIC)
        cpu.request(sfr.SSCRIC)
        self.b.plan()


class Can:
    """The on-chip CAN controller (the C167CR's CAN1, 0xEF00): control and
    status, bit timing, the masks, and fifteen message objects of eight
    words, as far as the firmware exercises them.

    A frame from the bus goes in with `receive(id, data)` and lands in the
    first receive object whose identifier matches: NEWDAT (0x0200) and INTPND
    (0x0002) set in its control word, and the interrupt register at 0xEF02
    answering with the object's number (object n is interrupt n+2, object 15
    is 2).  The interrupt is load-bearing: the ISR is what counts a reception
    into the shadow at 0xE752 + n*6, and can_rx_poll only reports a frame the
    ISR has counted.

    A transmit is what it is on the chip: a write to a message control
    register that sets TXRQ ('10' in bits 13-12) while CPUUPD (bits 11-10)
    ends up reset, '01' written or '11' (leave alone) over a reset one.
    The GS20 firmware ends its object update with 0xF7FF (CPUUPD reset) and
    then sends with 0xEFFF; the MS43's sends with one 0x6595.  The frame is
    on the bus a frame time later, `on_tx(id, data)` hears it, the busy bit
    the GS20's next send tests (0x8000) is clear again and TXRQ reads
    reset.  Neither firmware asks for a transmit interrupt, so the model
    raises none.  Everything else — timing, masks, arbitration,
    configuration, the data — is plain registers it writes and reads back."""

    TX_AT = 0.0002                         # an eight byte frame at 500 kbit/s, stuffed

    def __init__(self, board):
        self.b = board
        self.on_tx = None
        self.sent = []                     # (id, data) the module has put on the bus
        self.received = 0                  # frames that found an object
        self.unmatched = []                # frame ids nobody was listening for
        self.intq = []                     # pending answers for the interrupt register
        self.pending = []                  # (object, at) transmits waiting for their frame time
        self.next_at = NEVER
        for n in range(1, 16):
            board.cpu.hook_write(sfr.C1MCR1 + 0x10 * (n - 1), self._mcr_write(n))
        board.cpu.hook_read(sfr.C1IR, self._int_read)

    def reset(self):
        self.intq.clear()
        self.pending.clear()
        self.next_at = NEVER

    # ---- what the objects hold ------------------------------------------------
    def _mcr(self, n):
        return self.b.read16(0xEF00 + 0x10 * n)

    def _set_mcr(self, n, word):
        a = 0xEF00 + 0x10 * n
        self.b.cpu.low[a], self.b.cpu.low[a + 1] = word & 0xFF, word >> 8

    def ident(self, n):
        """The 11-bit identifier of object n, the way can_msg_object_copy decodes it."""
        a = 0xEF00 + 0x10 * n
        w = self.b.cpu.low[a + 2] | self.b.cpu.low[a + 3] << 8
        return (((w >> 8) | (w << 8)) >> 5) & 0x7FF

    def data(self, n):
        a = 0xEF00 + 0x10 * n
        length = self.b.cpu.low[a + 6] >> 4
        return bytes(self.b.cpu.low[a + 8:a + 8 + length])

    # ---- the register side the CPU sees --------------------------------------
    def _mcr_write(self, n):
        def write(addr, value):
            # The registers keep what is written, pairs and all; a '11'
            # pair leaves the bit as the register has it.
            cpuupd = (value if value & 0x0C00 != 0x0C00 else self._mcr(n)) & 0x0C00
            if value & 0x3000 == 0x2000 and cpuupd == 0x0400:
                at = self.b.cpu.instructions + int(self.TX_AT * self.b.ips)
                first = not self.pending
                # the message is taken as it stands: the firmware rewrites the
                # object for its next frame before this one is on the bus
                self.pending.append(((n, self.ident(n), self.data(n)), at))
                if first:                          # sends queue in order, so only the first moves it
                    self.next_at = at
                    self.b.plan()
            return value
        return write

    def _int_read(self, addr):
        """The interrupt register: one pending source a read, 0 when none are."""
        if not self.intq:
            return 0
        v = self.intq.pop(0)
        if self.intq:
            self.b.cpu.request(sfr.XP0IC)          # the ISR took one, another is due
        return v

    # ---- the bus side ---------------------------------------------------------
    def receive(self, ident, data):
        """A frame from the bus: into the highest receive object that matches it."""
        cpu = self.b.cpu
        for n in range(15, 0, -1):
            a = 0xEF00 + 0x10 * n
            cfg = cpu.low[a + 6]
            if not cfg or cfg & 0x08:              # never configured, or a transmit object
                continue
            if self.ident(n) != ident:
                continue
            d = bytes(data)[:8]
            cpu.low[a + 8:a + 8 + len(d)] = d
            self._set_mcr(n, self._mcr(n) | 0x0200 | 0x0002)
            self.intq.append(2 if n == 15 else n + 2)
            self.received += 1
            cpu.request(sfr.XP0IC)
            return True
        self.unmatched.append(ident)
        return False

    def service(self, now):
        still = [(frame, at) for frame, at in self.pending if now < at]
        for (n, ident, data), at in self.pending:
            if now >= at:
                if len(self.sent) < 1000:
                    self.sent.append((ident, data))
                if self.on_tx is not None:
                    self.on_tx(ident, data)
                self._set_mcr(n, (self._mcr(n) & ~0xB000) | 0x1000)    # not busy any more, TXRQ reset
        self.pending = still
        self.next_at = still[0][1] if still else NEVER
        self.b.plan()


class Adc:
    """The A/D converter: a conversion started with ADST is done a little
    later with the channel's value in ADDAT and an interrupt request.
    Channels read what `channels` says, 10 bits; `default` for the rest.
    `source(channel)`, when set, is asked first at each conversion's end
    (a board with a multiplexer in front of a channel answers by its
    address); None from it falls back to `channels`."""

    START, BUSY = 0x0080, 0x0100

    def __init__(self, board, default=0x300):
        self.b = board
        self.CONVERSION = board.ips // 10_000    # instructions: 100 us
        self.channels = {}
        self.default = default
        self.source = None
        self.conversions = 0
        self.channel = 0
        self.next_at = NEVER
        board.cpu.hook_write(sfr.ADCON, self._control)

    def _control(self, addr, con):
        if con & self.START and self.next_at == NEVER:
            self.channel = con & 0xF
            self.next_at = self.b.cpu.instructions + self.CONVERSION
            self.b.plan()
            return con | self.BUSY
        return con

    def service(self, now):
        if now < self.next_at:
            return
        cpu = self.b.cpu
        self.conversions += 1
        value = self.source(self.channel) if self.source is not None else None
        if value is None:
            value = self.channels.get(self.channel, self.default)
        cpu.ww(sfr.ADDAT, (self.channel << 12) | (int(value) & 0x3FF))
        cpu.request(sfr.ADCIC)
        con = cpu.rw(sfr.ADCON)
        mode = (con >> 4) & 3
        if mode & 2 and self.channel:            # auto scan: on down to channel 0
            self.channel -= 1
            self.next_at = now + self.CONVERSION
        elif mode & 1:                           # continuous
            self.channel = con & 0xF
            self.next_at = now + self.CONVERSION
        else:
            self.next_at = NEVER
            con &= ~(self.START | self.BUSY)
        cpu.low[sfr.ADCON], cpu.low[sfr.ADCON + 1] = con & 0xFF, (con >> 8) & 0xFF
        self.b.plan()


class Pwm:
    """The PWM unit: four 16-bit counters PT0-PT3, each clocked at fCPU or
    fCPU/64 (PWMCON0 PTIx) while PTRx runs, with a period PPx and a pulse
    width PWx. Edge-aligned (PWMCON1 PMx = 0) a counter goes 0..PP and
    starts again; centre-aligned it goes up to PP and back down to 0. At the
    end of each period PIRx is set and, with PIEx, PWMIC is requested. The
    outputs are not driven; `duty(n)` is the fraction of the period the
    channel's output is active, for a load model to read."""

    def __init__(self, board):
        self.b = board
        self.base = [0] * 4                      # the count at `stamp`
        self.stamp = [0] * 4
        self.running = [False] * 4
        self.next_at = NEVER
        self.periods = [0] * 4
        cpu = board.cpu
        cpu.hook_write(sfr.PWMCON0, self._control)
        cpu.hook_write(sfr.PWMCON1, self._mode)
        for n in range(4):
            cpu.hook_read(sfr.PT0 + 2 * n, lambda a, n=n: self._count(n))
            cpu.hook_write(sfr.PT0 + 2 * n, lambda a, v, n=n: self._set_count(n, v))
            cpu.hook_write(sfr.PP0 + 2 * n, lambda a, v, n=n: self._period_write(n, v))

    def _clock(self, n):
        """Counts a second."""
        fast = not (self.b.cpu.rw(sfr.PWMCON0) >> (4 + n)) & 1
        return self.b.fcpu if fast else self.b.fcpu / 64

    def _cycle(self, n):
        """Counts in one whole period."""
        pp = self.b.cpu.rw(sfr.PP0 + 2 * n) + 1
        centre = (self.b.cpu.rw(sfr.PWMCON1) >> (4 + n)) & 1
        return 2 * pp if centre else pp

    def _elapsed(self, n):
        """Counts since the period began, unwrapped."""
        if not self.running[n]:
            return self.base[n]
        return self.base[n] + int((self.b.cpu.instructions - self.stamp[n]) * self._clock(n) / self.b.ips)

    def _count(self, n):
        e = self._elapsed(n) % self._cycle(n)
        pp = self.b.cpu.rw(sfr.PP0 + 2 * n) + 1
        return e if e < pp else 2 * pp - e      # centre-aligned: on the way down

    def _rebase(self):
        for n in range(4):
            self.base[n], self.stamp[n] = self._elapsed(n) % self._cycle(n), self.b.cpu.instructions

    def _set_count(self, n, value):
        self.base[n], self.stamp[n] = value, self.b.cpu.instructions
        self._plan()
        return value

    def _period_write(self, n, value):
        self._rebase()
        self.b.cpu.low[sfr.PP0 + 2 * n], self.b.cpu.low[sfr.PP0 + 2 * n + 1] = value & 0xFF, value >> 8
        self._plan()
        return value

    def _control(self, addr, con):
        self._rebase()
        self.b.cpu.low[addr], self.b.cpu.low[addr + 1] = con & 0xFF, con >> 8
        for n in range(4):
            self.running[n] = bool(con >> n & 1)
        self._plan()
        return con

    def _mode(self, addr, con):
        self._rebase()
        self.b.cpu.low[addr], self.b.cpu.low[addr + 1] = con & 0xFF, con >> 8
        self._plan()
        return con

    def _period_end(self, n):
        """The instruction at which channel n's current period ends."""
        left = self._cycle(n) - self.base[n] % self._cycle(n)
        return self.stamp[n] + max(1, -int(-left * self.b.ips // self._clock(n)))

    def _plan(self):
        con = self.b.cpu.rw(sfr.PWMCON0)
        ends = [self._period_end(n) for n in range(4) if self.running[n] and con >> (8 + n) & 1]
        self.next_at = min(ends) if ends else NEVER
        self.b.plan()

    def duty(self, n):
        cpu = self.b.cpu
        pp = cpu.rw(sfr.PP0 + 2 * n) + 1
        return max(0.0, min(1.0, (pp - cpu.rw(sfr.PW0 + 2 * n)) / pp))

    def service(self, now):
        if now < self.next_at:
            return
        cpu = self.b.cpu
        con = cpu.rw(sfr.PWMCON0)
        for n in range(4):
            if self.running[n] and con >> (8 + n) & 1 and self._period_end(n) <= now:
                self.periods[n] += 1
                self.base[n], self.stamp[n] = 0, self._period_end(n)
                con |= 1 << (12 + n)
        cpu.low[sfr.PWMCON0], cpu.low[sfr.PWMCON0 + 1] = con & 0xFF, con >> 8
        cpu.request(sfr.PWMIC)
        self._plan()


class FlashChip:
    """An external flash, an Am29F400BB: the AMD command set as the boot
    block's driver uses it (unlock cycles at byte addresses 0xAAA and
    0x555 on an 8-bit bus, 0xAAAA and 0x5554 on a 16-bit one), autoselect, program, sector and chip erase, reset). Programming
    and erasing take no time; the driver's first look finds them done.
    Programming can only clear bits, as on the part."""

    MANUFACTURER, DEVICE = 0x0001, 0x22AB
    # bottom boot: 16K, 8K, 8K, 32K, then 64K sectors
    SECTORS = [(0x00000, 0x4000), (0x04000, 0x2000), (0x06000, 0x2000), (0x08000, 0x8000)] + [(a, 0x10000) for a in range(0x10000, 0x80000, 0x10000)]

    def __init__(self, board):
        self.b = board
        self.state = 0                           # unlock cycles seen
        self.erase_armed = False
        self.autoselect = False
        self.programmed = 0
        self.erased = []                         # sector starts, in the order they were erased
        self.commands = []
        board.cpu.on_flash_write = self.write
        self.image = board.cpu.flash

    def _array(self):
        self.autoselect = False
        self.b.cpu.flash_read = None

    def _id(self, offset, size):
        word = {0: self.MANUFACTURER, 1: self.DEVICE}.get((offset & 0xFF) >> 1, 0)
        if size == 2:
            return word
        return word >> 8 if offset & 1 else word & 0xFF

    def sector(self, offset):
        for start, size in self.SECTORS:
            if start <= offset < start + size:
                return start, size

    def write(self, addr, value, size):
        offset = addr & 0x7FFFF
        low = offset & 0xFFF
        data = value & 0xFF
        if self.state == 3:                      # the program command's data
            self.state = 0
            for i in range(size):
                self.image[offset + i] &= (value >> (8 * i)) & 0xFF
            self.programmed += size
            return
        if data == 0xF0:
            self.state, self.erase_armed = 0, False
            self._array()
            return
        if self.state == 0 and low == 0xAAA and data == 0xAA:
            self.state = 1
        elif self.state == 1 and low in (0x555, 0x554) and data == 0x55:   # 0x554: word 0x2AA on a 16-bit bus (MS43)
            self.state = 2
        elif self.state == 2:
            self.state = 0
            self.commands.append(data)
            if self.erase_armed:
                self.erase_armed = False
                if data == 0x30:
                    start, length = self.sector(offset)
                    self.image[start:start + length] = b"\xff" * length
                    self.erased.append(start)
                elif data == 0x10 and low == 0xAAA:
                    self.image[:] = b"\xff" * len(self.image)
                    self.erased.append(-1)
            elif low == 0xAAA and data == 0x90:
                self.autoselect = True
                self.b.cpu.flash_read = self._id
            elif low == 0xAAA and data == 0xA0:
                self.state = 3
            elif low == 0xAAA and data == 0x80:
                self.erase_armed = True
        else:
            self.state = 0                       # 0x30 on its own (erase resume) and anything stray


class CapcomTimers:
    """A CAPCOM unit: its timers T0 and T1 (with `second_unit`, T7 and T8 of
    the second CAPCOM, and CC16-CC31 for CC0-CC15), and its compare
    channels.

    The timers count the CPU clock through a prescaler, fCPU/8 << TxI,
    always up; at the overflow they load their reload register and request
    their interrupt. The control word is shared, T01CON (T78CON), low byte
    T0 (T7), high byte T1 (T8). In counter mode (TxM) T0 and T7 count
    events instead: TxI x00 the overflows/underflows of GPT2's T6 (the
    board calls `t6_wrapped()`), x01/x10/x11 the rising/falling/either
    edges of their input pin (`count_edge(level)`, from whoever drives
    T0IN P3.0 or T7IN P2.15). Their compare channels match on the count,
    and on a write of the timer that makes it equal to the compare value.
    T1 and T8 in counter mode are noted in board.unsupported.

    A channel in a compare mode (CCMODx 4-7 in CCM0-CCM3, CCM4-CCM7)
    requests its interrupt when its timer (ACCx: T0/T7 or T1/T8) counts to
    the value in its CCx register, once each time round; `on_compare(n,
    mode)` hears each match, with the channel's number (0-31). The pins
    are not driven.

    A channel in a capture mode (CCMODx 1 rising, 2 falling, 3 both edges)
    latches its timer into CCx and requests its interrupt when its input
    pin changes the way the mode says: whoever drives the pin calls
    `edge(n, level)`. The first unit's CC0-CC15 pins are port P2's, so a
    read of P2 returns the driven levels on the bits DP2 leaves as inputs."""

    RUN, COUNTER = 0x40, 0x08

    def __init__(self, board, second_unit=False):
        self.b = board
        names = ("T7", "T8") if second_unit else ("T0", "T1")
        self.con_at = sfr.T78CON if second_unit else sfr.T01CON
        self.timers = []
        for shift, name in zip((0, 8), names):
            t = _CapcomTimer(board, name, shift)
            t.unit = self
            self.timers.append(t)
        cpu = board.cpu
        cpu.hook_write(self.con_at, self._control)
        first = 16 if second_unit else 0
        self.channels = [_CompareChannel(self, first + i) for i in range(16)]
        for i in range(4):
            cpu.hook_write(getattr(sfr, f"CCM{i + (4 if second_unit else 0)}"), self._mode(i))
        for ch in self.channels:
            cpu.hook_write(ch.cc_at, ch.write)
        self.on_compare = None
        self.compares = 0
        self.captures = 0
        self.next_at = NEVER
        # the levels driven on the channels' pins, 1 high (n -> level)
        self.pins = {}
        if not second_unit:
            cpu.hook_read(sfr.P2, self._read_p2)

    def t6_wrapped(self):
        """GPT2's T6 overflowed or underflowed: T0 (T7) counts it if it
        is in counter mode with input x00."""
        t = self.timers[0]
        if t.counting and t.input & 3 == 0:
            t.count()

    def count_edge(self, level):
        """The T0IN (T7IN) pin went to `level`: T0 (T7) counts it if its
        counter mode takes that edge."""
        t = self.timers[0]
        if not t.counting:
            return
        sel = t.input & 3
        if sel == 3 or (sel == 1 and level) or (sel == 2 and not level):
            t.count()

    def _read_p2(self, addr):
        """P2: the latch on the output bits, the driven pin levels on the input ones."""
        cpu = self.b.cpu
        latch, out = cpu.low[sfr.P2] | (cpu.low[sfr.P2 + 1] << 8), cpu.rw(sfr.DP2)
        level = 0
        for n, high in self.pins.items():
            if high and n < 16:
                level |= 1 << n
        return (latch & out) | (level & ~out & 0xFFFF)

    def edge(self, n, level):
        """The pin of channel n is driven to `level` (truthy high). A
        capture channel latches its timer if the change is one its mode
        captures."""
        level = 1 if level else 0
        first = self.channels[0].n
        was = self.pins.get(n, 1)
        self.pins[n] = level
        if was == level:
            return
        ch = self.channels[n - first]
        if not ch.capture or ch.timer is None:
            return
        rising = level == 1
        if (ch.capture == 1 and not rising) or (ch.capture == 2 and rising):
            return
        value = ch.timer.now()
        cpu = self.b.cpu
        cpu.low[ch.cc_at], cpu.low[ch.cc_at + 1] = value & 0xFF, value >> 8
        self.captures += 1
        cpu.request(ch.ic)

    def _control(self, addr, con):
        for t in self.timers:
            t.control(con)
        self._replan()
        return con

    def _mode(self, i):
        def write(addr, value):
            for k in range(4):
                self.channels[4 * i + k].configure((value >> (4 * k)) & 0xF)
            self._plan()
            return value
        return write

    def _replan(self):
        """A timer changed: every channel's next match moves with it."""
        now = self.b.cpu.instructions
        for ch in self.channels:
            ch.plan(now)
        self._plan()

    def _plan(self):
        self.next_at = min(min(t.next_at for t in self.timers), min(ch.next_at for ch in self.channels))
        self.b.plan()

    def reset(self):
        """A CPU reset: the timers stop, the channels go back to disabled."""
        for t in self.timers:
            t.reset()
        for ch in self.channels:
            ch.mode, ch.capture, ch.timer, ch.next_at = 0, 0, None, NEVER
        self.next_at = NEVER

    def service(self, now):
        for ch in self.channels:
            if now >= ch.next_at:
                ch.match(now)
        wrapped = False
        for t in self.timers:
            if now >= t.next_at:
                t.overflow()
                wrapped = True
        if wrapped:
            self._replan()
        else:
            self._plan()


class _CompareChannel:
    def __init__(self, unit, n):
        self.unit = unit
        self.n = n
        self.cc_at = getattr(sfr, f"CC{n}")
        self.ic = getattr(sfr, f"CC{n}IC")
        self.mode = 0
        self.capture = 0                # 1 rising, 2 falling, 3 both; 0 not capturing
        self.timer = None
        self.next_at = NEVER

    def configure(self, nibble):
        mode = nibble & 7
        self.mode = mode if mode >= 4 else 0
        self.capture = mode if 1 <= mode <= 3 else 0
        self.timer = self.unit.timers[nibble >> 3]
        self.plan(self.unit.b.cpu.instructions)

    def write(self, addr, value):
        self.plan(self.unit.b.cpu.instructions, value)
        self.unit._plan()
        return value

    def plan(self, now, value=None):
        """When the timer next counts to the CCx value, after `now`."""
        t = self.timer
        if not self.mode or t is None or not t.running:
            self.next_at = NEVER          # (a counting timer's matches come from count())
            return
        cc = self.unit.b.cpu.low[self.cc_at] | (self.unit.b.cpu.low[self.cc_at + 1] << 8) if value is None else value
        at = t.stamp + (cc - t.value) * t.step if cc > t.value else NEVER
        if at <= now or at >= t.next_at:
            # passed this time round: the next one, from the reload value
            reload = self.unit.b.cpu.rw(t.rel_at)
            at = t.next_at + (cc - reload) * t.step if cc >= reload and t.next_at != NEVER else NEVER
        self.next_at = at

    def match(self, now):
        self.unit.compares += 1
        self.unit.b.cpu.request(self.ic)
        if self.unit.on_compare is not None:
            self.unit.on_compare(self.n, self.mode)
        t = self.timer
        reload = self.unit.b.cpu.rw(t.rel_at)
        cc = self.unit.b.cpu.low[self.cc_at] | (self.unit.b.cpu.low[self.cc_at + 1] << 8)
        self.next_at = t.next_at + (cc - reload) * t.step if cc >= reload and t.next_at != NEVER else NEVER


class _CapcomTimer:
    def __init__(self, board, name, shift):
        self.b = board
        self.name = name
        self.shift = shift
        self.value_at, self.rel_at, self.ic = getattr(sfr, name), getattr(sfr, name + "REL"), getattr(sfr, name + "IC")
        self.base_step = 8 * board.ips // board.fcpu          # instructions per count at the fastest prescaler
        assert self.base_step >= 1, f"{name} cannot count at {board.ips} instructions a second"
        self.value, self.stamp, self.step, self.running = 0, 0, self.base_step, False
        self.counting = False            # running in counter mode
        self.input = 0                   # TxI
        self.next_at = NEVER
        self.wraps = 0
        cpu = board.cpu
        cpu.hook_read(self.value_at, lambda a: self.now())
        cpu.hook_write(self.value_at, self._set_value)

    def now(self):
        if not self.running:
            return self.value
        return (self.value + (self.b.cpu.instructions - self.stamp) // self.step) & 0xFFFF

    def _plan(self):
        self.next_at = self.stamp + (0x10000 - self.value) * self.step if self.running else NEVER

    def _set_value(self, addr, value):
        self.value, self.stamp = value, self.b.cpu.instructions
        self._plan()
        self.unit._replan()
        # a write that makes the timer equal to a compare register is a
        # match too: the MS43's gap handling moves T7 onto its CC24 value
        # by software (0x0E4E90) and waits for that match to wrap the angle
        self._matches(value)
        return value

    def _matches(self, value):
        """Request the interrupt of each compare channel on this timer whose
        CCx equals `value`."""
        cpu = self.b.cpu
        for ch in self.unit.channels:
            if ch.mode and ch.timer is self:
                cc = cpu.low[ch.cc_at] | (cpu.low[ch.cc_at + 1] << 8)
                if cc == value:
                    self.unit.compares += 1
                    cpu.request(ch.ic)
                    if self.unit.on_compare is not None:
                        self.unit.on_compare(ch.n, ch.mode)

    def control(self, con):
        con = (con >> self.shift) & 0xFF
        self.value, self.stamp = self.now(), self.b.cpu.instructions
        run, counter = bool(con & CapcomTimers.RUN), bool(con & CapcomTimers.COUNTER)
        self.running = run and not counter
        self.input = con & 7
        # T0/T7 count events; T1/T8 in counter mode count T0/T7's overflows
        # (not modelled)
        self.counting = run and counter and self.shift == 0
        if run and counter and self.shift:
            self.b.unsupported.add(f"{self.name} counter mode")
        self.step = self.base_step << (con & 7)
        self._plan()

    def count(self):
        """One counted event: up by one, the reload and the interrupt at the
        overflow, and the compare channels on this timer that the new count
        matches."""
        cpu = self.b.cpu
        if self.value == 0xFFFF:
            self.wraps += 1
            self.value = cpu.rw(self.rel_at)
            cpu.request(self.ic)
        else:
            self.value += 1
        self._matches(self.value)

    def overflow(self):
        self.wraps += 1
        self.stamp = self.next_at
        self.value = self.b.cpu.rw(self.rel_at)
        self.b.cpu.request(self.ic)
        self._plan()

    def reset(self):
        self.running, self.counting, self.next_at, self.value = False, False, NEVER, 0
