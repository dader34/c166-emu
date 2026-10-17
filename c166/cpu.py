"""A C167 core: the instruction set of the C166 family, interpreted.

Unicorn has no C166, so this is the whole CPU: registers, the memory map
as far as the CPU sees it, and one handler per opcode. The encodings and
flag rules follow Infineon's "Instruction Set Manual for the C166 Family"
(V2.0, 2001-03). Written for the GS20 (gs20-emu) and shared with the MS43
(ms43-emu); what differs between boards is where the flash and the
external RAM are, which a board says with `map_flash` and `map_ram`.

Segment 0 is always here: the flash from its start up to `mirror_end`
(where the trap vectors are), and above it RAM, XRAM, the CAN module, the
ESFRs, the internal RAM and the SFRs, all memory until something hooks
them. Above segment 0 the address space is mapped in 4 KB pages, each
open (reads 0xFFFF, counted in `open_reads`), flash (the chip's bytes at
the address masked to its size, so a chip that ignores its upper address
lines shows up in every window decoded to it), or external RAM. The
default is the GS20's: flash wherever address bit 19 is set.

Data addresses are 16 bits wide and go through the data page pointers:
the top two bits pick DPP0-3, whose 10 bits become address bits 14-23.
EXTP/EXTS replace that for the next one to four instructions, and EXTR
turns the short register addresses into ESFR ones.

Interrupts are the CPU's own business and are here: a peripheral calls
request() with the address of its interrupt control register, and the
request is taken between two instructions when its level beats the PSW's,
through a PEC channel if it is at level 14 or 15 and the channel still has
transfers to do. Time is the instruction count: one instruction is two CPU
clocks. Devices ask to be called at a count with `next_event`/`on_event`.

An undefined opcode or a word access at an odd address stops the emulation
with CpuError, which is what one wants while testing a patch. With
`hardware_traps` set they do what the chip does instead: the class B trap
(vector 0x0A) with the cause in TFR, which in this firmware ends in a
software reset. The stack overflow and underflow traps are not here, and
neither are the peripherals themselves: every SFR is memory until
something hooks it (board.py does).
"""

# PSW
N, C, V, Z, E = 0x01, 0x02, 0x04, 0x08, 0x10
MULIP, USR0, HLDEN, IEN = 0x20, 0x40, 0x400, 0x800
FLAGS = N | C | V | Z | E

# SFRs the core itself lives in
DPP0, CSP, MDH, MDL, CP, SP, STKOV, STKUN = 0xFE00, 0xFE08, 0xFE0C, 0xFE0E, 0xFE10, 0xFE12, 0xFE14, 0xFE16
MDC, PSW, SYSCON, ZEROS, ONES = 0xFF0E, 0xFF10, 0xFF12, 0xFF1C, 0xFF1E
TFR, CLASS_B_TRAP = 0xFFAC, 0x0A
UNDOPC, ILLOPA = 0x80, 0x04                      # TFR: undefined opcode, illegal word operand access
MDRIU = 0x10
SGTDIS = 0x0800

# Interrupt control registers (xxIC) and the trap each one's request enters
IC_TRAPS = {0xFF9C: 0x20, 0xFF9E: 0x21, 0xF17A: 0x3D, 0xF17C: 0x3E,                    # T0 T1 T7 T8
            0xFF60: 0x22, 0xFF62: 0x23, 0xFF64: 0x24, 0xFF66: 0x25, 0xFF68: 0x26,      # T2-T6
            0xFF6A: 0x27, 0xFF98: 0x28, 0xFF9A: 0x29,                                  # CAPREL, ADC, ADC error
            0xFF6C: 0x2A, 0xFF6E: 0x2B, 0xFF70: 0x2C, 0xF19C: 0x47,                    # ASC0: transmit, receive, error, buffer
            0xFF72: 0x2D, 0xFF74: 0x2E, 0xFF76: 0x2F,                                  # SSC
            0xF17E: 0x3F, 0xF186: 0x40, 0xF18E: 0x41, 0xF196: 0x42, 0xF19E: 0x43,      # PWM, XP0 (CAN), XP1-XP3
            0xF178: 0x3C, 0xF184: 0x44, 0xF18C: 0x45, 0xF194: 0x46}                    # CC28-CC31
IC_TRAPS.update({0xFF78 + 2 * i: 0x10 + i for i in range(16)})                        # CC0-CC15
IC_TRAPS.update({0xF160 + 2 * i: 0x30 + i for i in range(12)})                        # CC16-CC27
IR, IE = 0x80, 0x40
PECC0, SRCP0 = 0xFEC0, 0xFCE0                    # PEC channel n: control at PECC0 + 2n, pointers at SRCP0 + 4n
NEVER = 1 << 62

FLASH_SIZE = 0x80000
FLASH_MIRROR_END = 0xC000        # 0x000000-0x00BFFF shows the flash from its start
RAM_KIND, HOOK_KIND, FLASH_KIND = 0, 1, 2
OPEN_PAGE, FLASH_PAGE, RAM_PAGE, HOOK_PAGE = 0, 1, 2, 3   # what a 4 KB page above segment 0 holds;
# a hook page is RAM with a device watching writes to some of its addresses (hook_ext_write)
PAGES = 0x1000                   # 16 MB in 4 KB pages
EXT_RAM_LIMIT = 0x100000         # external RAM is only mapped below 1 MB


class CpuError(Exception):
    pass


class _Fault(CpuError):
    """Something the chip answers with a class B trap."""

    def __init__(self, flag, message):
        super().__init__(message)
        self.flag = flag


def _conditions():
    """cond[cc][psw & 0x1F]"""
    table = []
    for cc in range(16):
        row = []
        for f in range(32):
            n, c, v, z, e = bool(f & N), bool(f & C), bool(f & V), bool(f & Z), bool(f & E)
            row.append([True, not (z or e), z, not z, v, not v, n, not n, c, not c,
                        not z and n == v, z or n != v, n != v, n == v, not (z or c), z or c][cc])
        table.append(row)
    return table


COND = _conditions()
# Four bytes when the low nibble of the opcode is one of these, two otherwise.
LENGTH = [4 if (op & 0xF) in (2, 3, 4, 5, 6, 7, 0xA) else 2 for op in range(256)]


class Cpu:
    def __init__(self, flash_size=FLASH_SIZE, mirror_end=FLASH_MIRROR_END, default_map=True):
        """default_map: flash wherever address bit 19 is set (the GS20).
        Without it nothing is mapped above segment 0 until the board
        calls map_flash and map_ram."""
        self.low = bytearray(0x10000)            # segment 0 above the flash mirror: RAM, SFRs
        self.flash = bytearray(b"\xff" * flash_size)
        self.flash_mask = flash_size - 1
        assert flash_size & self.flash_mask == 0, "the flash size must be a power of two"
        self.kind = bytearray(0x10000)
        self.kind[:mirror_end] = bytes([FLASH_KIND]) * mirror_end
        self.ext_writers = {}                    # external RAM address -> fn(addr, value, size), see hook_ext_write
        self.pages = bytearray(PAGES)            # page (address >> 12) -> OPEN_PAGE / FLASH_PAGE / RAM_PAGE
        self.ext_ram = bytearray(EXT_RAM_LIMIT)  # backing for RAM pages, by physical address
        if default_map:
            self.map_flash(0, PAGES << 12, only_if=lambda a: a & 0x80000)
        self.readers = {}                        # even address -> fn(addr) -> word or None
        self.writers = {}                        # even address -> fn(addr, word) -> word to store or None
        self.on_flash_write = None               # fn(addr, value, size): the flash chip's command interface
        self.flash_read = None                   # fn(offset, size): set while the chip shows something other than its array
        self.flash_writes = []
        self.open_reads = {}                     # addresses with nothing behind them that were read
        self.pc_hooks = {}                       # pc -> fn(cpu), called before the instruction there
        self.instructions = 0
        self.at = 0                              # the instruction being executed (pc is already past it)
        self.next_event = NEVER                  # instruction count at which on_event() is due
        self.on_event = None
        self.on_reset = None                     # the peripherals' share of a reset
        self.attention = False                   # an interrupt request may be ready to be taken
        self.requests = set()                    # xxIC addresses with their request and enable bits both set
        self.interrupts_taken = {}               # trap number -> count
        self.hardware_traps = False              # True: a fault enters the class B trap instead of raising CpuError
        self.faults = []                         # (instruction address, TFR flag, what) of the faults trapped
        self.pec_transfers = 0
        self.idle = False
        self.watchdog_services = 0
        self.watchdog_disabled = False
        self.einit = False
        self.resets = 0
        self.ops = [self._undefined] * 256
        self._build()
        self._core_sfrs()
        self.reset()

    # ---- reset ---------------------------------------------------------------
    def reset(self):
        self.csp, self.ip = 0, 0
        self.psw = 0
        self.sp = 0xFC00
        self.cp = 0xFC00
        self.dpp = [0 << 14, 1 << 14, 2 << 14, 3 << 14]
        low = self.low
        low[0xEF00:0xF200] = bytes(0x300)        # the CAN module and the ESFRs
        low[0xFE00:0x10000] = bytes(0x200)       # the SFRs
        for i in range(4):
            low[DPP0 + 2 * i], low[DPP0 + 2 * i + 1] = i, 0
        low[CP], low[CP + 1] = 0x00, 0xFC
        low[STKOV], low[STKOV + 1] = 0x00, 0xFA
        low[STKUN], low[STKUN + 1] = 0x00, 0xFC
        self.ext = 0                             # instructions left in an ATOMIC/EXT sequence
        self.ext_r = False                       # EXTR: reg and bitoff address the ESFR area
        self.ext_mode = 0                        # 1: page override, 2: segment override
        self.ext_base = 0
        self.idle = False
        self.requests.clear()
        for ic in IC_TRAPS:
            low[ic] = low[ic + 1] = 0
        if self.on_reset is not None:
            self.on_reset()

    def _core_sfrs(self):
        def set_psw(a, v):
            self.psw = v
            self.attention = True                # the level or the enable may have changed
            return v

        def set_ic(a, v):
            if v & (IR | IE) == IR | IE:
                self.requests.add(a)
                self.attention = True
            else:
                self.requests.discard(a)
            return v & 0xFF

        for ic in IC_TRAPS:
            self.hook_write(ic, set_ic)

        def set_sp(a, v):
            self.sp = v & 0xFFFE
            return self.sp

        def set_cp(a, v):
            self.cp = v & 0xFFFE
            return self.cp

        def set_dpp(a, v):
            self.dpp[(a - DPP0) >> 1] = (v & 0x3FF) << 14
            return v & 0x3FF

        self.hook_read(PSW, lambda a: self.psw)
        self.hook_write(PSW, set_psw)
        self.hook_read(SP, lambda a: self.sp)
        self.hook_write(SP, set_sp)
        self.hook_write(CP, set_cp)
        for i in range(4):
            self.hook_write(DPP0 + 2 * i, set_dpp)
        self.hook_read(CSP, lambda a: self.csp)
        self.hook_write(CSP, lambda a, v: self.csp)          # only branches change it
        self.hook_read(ZEROS, lambda a: 0)
        self.hook_write(ZEROS, lambda a, v: 0)
        self.hook_read(ONES, lambda a: 0xFFFF)
        self.hook_write(ONES, lambda a, v: 0xFFFF)

    # ---- the memory map above segment 0 ---------------------------------------
    def map_flash(self, start, end, only_if=None):
        """Show the flash in [start, end) (4 KB aligned), at the address masked to its size."""
        for page in range(max(start, 0x10000) >> 12, end >> 12):
            if only_if is None or only_if(page << 12):
                self.pages[page] = FLASH_PAGE

    def map_ram(self, start, size):
        """External RAM in [start, start + size), below 1 MB (4 KB aligned)."""
        assert 0x10000 <= start and start + size <= EXT_RAM_LIMIT and not (start | size) & 0xFFF
        for page in range(start >> 12, (start + size) >> 12):
            self.pages[page] = RAM_PAGE

    def unmap(self, start, end):
        for page in range(max(start, 0x10000) >> 12, end >> 12):
            self.pages[page] = OPEN_PAGE

    # ---- hooks ---------------------------------------------------------------
    def hook_read(self, addr, fn):
        self.readers[addr] = fn
        self.kind[addr] = self.kind[addr + 1] = HOOK_KIND

    def hook_write(self, addr, fn):
        self.writers[addr] = fn
        self.kind[addr] = self.kind[addr + 1] = HOOK_KIND

    def hook_ext_write(self, addr, fn):
        """fn(addr, value, size) after a write to external RAM at `addr`
        (an even address; a byte or word write there). The page must be RAM."""
        page = addr >> 12
        assert self.pages[page] in (RAM_PAGE, HOOK_PAGE), f"0x{addr:06X} is not external RAM"
        self.pages[page] = HOOK_PAGE
        self.ext_writers[addr] = fn

    # ---- memory, by physical address -------------------------------------------
    def rb(self, a):
        if a < 0x10000:
            k = self.kind[a]
            if not k:
                return self.low[a]
            if k == FLASH_KIND:
                return self.flash[a] if self.flash_read is None else self.flash_read(a, 1)
            w = self._hooked_rw(a & 0xFFFE)
            return w >> 8 if a & 1 else w & 0xFF
        p = self.pages[(a >> 12) & 0xFFF]
        if p == FLASH_PAGE:
            o = a & self.flash_mask
            return self.flash[o] if self.flash_read is None else self.flash_read(o, 1)
        if p == RAM_PAGE or p == HOOK_PAGE:
            return self.ext_ram[a]
        return self._open(a) & 0xFF

    def rw(self, a):
        if a & 1:
            raise _Fault(ILLOPA, f"word read at odd address 0x{a:06X} (instruction at 0x{self.at:06X})")
        if a < 0x10000:
            k = self.kind[a]
            if not k:
                low = self.low
                return low[a] | (low[a + 1] << 8)
            if k == FLASH_KIND:
                return self.flash[a] | (self.flash[a + 1] << 8) if self.flash_read is None else self.flash_read(a, 2)
            return self._hooked_rw(a)
        p = self.pages[(a >> 12) & 0xFFF]
        if p == FLASH_PAGE:
            o = a & self.flash_mask
            return self.flash[o] | (self.flash[o + 1] << 8) if self.flash_read is None else self.flash_read(o, 2)
        if p == RAM_PAGE or p == HOOK_PAGE:
            return self.ext_ram[a] | (self.ext_ram[a + 1] << 8)
        return self._open(a)

    def wb(self, a, v):
        if a < 0x10000:
            k = self.kind[a]
            if not k:
                self.low[a] = v & 0xFF
            elif k == FLASH_KIND:
                self._flash_write(a, v & 0xFF, 1)
            else:
                e = a & 0xFFFE
                w = self._hooked_rw(e)
                self._hooked_ww(e, (w & 0x00FF) | ((v & 0xFF) << 8) if a & 1 else (w & 0xFF00) | (v & 0xFF))
        else:
            p = self.pages[(a >> 12) & 0xFFF]
            if p == FLASH_PAGE:
                self._flash_write(a, v & 0xFF, 1)
            elif p == RAM_PAGE:
                self.ext_ram[a] = v & 0xFF
            elif p == HOOK_PAGE:
                self.ext_ram[a] = v & 0xFF
                fn = self.ext_writers.get(a)
                if fn is not None:
                    fn(a, v & 0xFF, 1)

    def ww(self, a, v):
        if a & 1:
            raise _Fault(ILLOPA, f"word write at odd address 0x{a:06X} (instruction at 0x{self.at:06X})")
        if a < 0x10000:
            k = self.kind[a]
            if not k:
                low = self.low
                low[a] = v & 0xFF
                low[a + 1] = (v >> 8) & 0xFF
            elif k == FLASH_KIND:
                self._flash_write(a, v & 0xFFFF, 2)
            else:
                self._hooked_ww(a, v & 0xFFFF)
        else:
            p = self.pages[(a >> 12) & 0xFFF]
            if p == FLASH_PAGE:
                self._flash_write(a, v & 0xFFFF, 2)
            elif p == RAM_PAGE:
                self.ext_ram[a] = v & 0xFF
                self.ext_ram[a + 1] = (v >> 8) & 0xFF
            elif p == HOOK_PAGE:
                self.ext_ram[a] = v & 0xFF
                self.ext_ram[a + 1] = (v >> 8) & 0xFF
                fn = self.ext_writers.get(a)
                if fn is not None:
                    fn(a, v & 0xFFFF, 2)

    def _hooked_rw(self, a):
        fn = self.readers.get(a)
        if fn is not None:
            v = fn(a)
            if v is not None:
                return v & 0xFFFF
        return self.low[a] | (self.low[a + 1] << 8)

    def _hooked_ww(self, a, v):
        fn = self.writers.get(a)
        if fn is not None:
            r = fn(a, v)
            if r is not None:
                v = r
        self.low[a] = v & 0xFF
        self.low[a + 1] = (v >> 8) & 0xFF

    def _flash_write(self, a, v, size):
        if self.on_flash_write is not None:
            self.on_flash_write(a, v, size)
        elif len(self.flash_writes) < 10000:
            self.flash_writes.append((a, v, size, self.at))

    def _open(self, a):
        self.open_reads[a] = self.open_reads.get(a, 0) + 1
        return 0xFFFF

    def read(self, a, size):
        return bytes(self.rb(a + i) for i in range(size))

    def write(self, a, data):
        for i, b in enumerate(data):
            self.wb(a + i, b)

    # ---- registers -------------------------------------------------------------
    @property
    def pc(self):
        return (self.csp << 16) | self.ip

    def r(self, n):
        o = self.cp + 2 * n
        low = self.low
        return low[o] | (low[o + 1] << 8)

    def set_r(self, n, v):
        o = self.cp + 2 * n
        low = self.low
        low[o] = v & 0xFF
        low[o + 1] = (v >> 8) & 0xFF

    def rbyte(self, n):
        """RL0, RH0, RL1, ... by index."""
        return self.low[self.cp + n]

    def set_rbyte(self, n, v):
        self.low[self.cp + n] = v & 0xFF

    def da(self, a16):
        """A 16-bit data address as the CPU resolves it."""
        mode = self.ext_mode
        if not mode:
            return self.dpp[a16 >> 14] | (a16 & 0x3FFF)
        if mode == 1:
            return self.ext_base | (a16 & 0x3FFF)
        return self.ext_base | a16

    # `reg`: a GPR for 0xF0-0xFF, otherwise the SFR (or ESFR) at twice the value
    def _reg_addr(self, rr):
        return (0xF000 if self.ext_r else 0xFE00) + 2 * rr

    def reg_rw(self, rr):
        if rr >= 0xF0:
            return self.r(rr & 0xF)
        return self.rw(self._reg_addr(rr))

    def reg_ww(self, rr, v):
        if rr >= 0xF0:
            self.set_r(rr & 0xF, v)
        else:
            self.ww(self._reg_addr(rr), v)

    def reg_rb(self, rr):
        if rr >= 0xF0:
            return self.low[self.cp + (rr & 0xF)]
        return self.rb(self._reg_addr(rr))

    def reg_wb(self, rr, v):
        if rr >= 0xF0:
            self.low[self.cp + (rr & 0xF)] = v & 0xFF
        else:
            self.ww(self._reg_addr(rr), v & 0xFF)     # a byte written to an SFR clears its other byte

    def _bit_addr(self, qq):
        if qq < 0x80:
            return 0xFD00 + 2 * qq
        if qq < 0xF0:
            return (0xF100 if self.ext_r else 0xFF00) + 2 * (qq & 0x7F)
        return self.cp + 2 * (qq & 0xF)

    # ---- the system stack ---------------------------------------------------------
    def push(self, v):
        self.sp = (self.sp - 2) & 0xFFFF
        self.ww(self.sp, v)

    def pop(self):
        v = self.rw(self.sp)
        self.sp = (self.sp + 2) & 0xFFFF
        return v

    def trap(self, number):
        """Enter a trap or interrupt vector the way the hardware does."""
        self.push(self.psw)
        if not self.rw(SYSCON) & SGTDIS:
            self.push(self.csp)
        self.push(self.ip)
        self.csp, self.ip = 0, (number * 4) & 0xFFFF
        self.idle = False

    # ---- interrupts ---------------------------------------------------------------
    def request(self, ic):
        """A peripheral's interrupt request: sets xxIR in the control register at `ic`."""
        low = self.low
        low[ic] |= IR
        if low[ic] & IE:
            self.requests.add(ic)
            self.attention = True

    def _service(self):
        """Take the best pending request if the PSW lets it in. Called between instructions."""
        if self.ext:
            return                               # not inside an ATOMIC or EXT sequence: look again after it
        self.attention = False
        if not self.requests or not self.psw & IEN:
            return
        low = self.low
        ic = max(self.requests, key=lambda a: low[a] & 0x3F)
        level = (low[ic] >> 2) & 0xF
        if level <= self.psw >> 12:
            return
        if level >= 14:
            channel = ((level & 1) << 2) | (low[ic] & 3)
            if self._pec(channel):
                low[ic] &= ~IR & 0xFF
                self.requests.discard(ic)
                self.attention = bool(self.requests)
                return
        low[ic] &= ~IR & 0xFF
        self.requests.discard(ic)
        number = IC_TRAPS[ic]
        self.interrupts_taken[number] = self.interrupts_taken.get(number, 0) + 1
        self.trap(number)
        self.psw = (self.psw & 0x0FFF) | (level << 12)
        self.attention = bool(self.requests)

    def _pec(self, channel):
        """One transfer of a PEC channel. False when its count is used up and
        the interrupt routine is to run instead; the transfer that brings
        the count to zero leaves the request standing for that."""
        ctl = self.rw(PECC0 + 2 * channel)
        count = ctl & 0xFF
        if not count:
            return False
        sp, dp = SRCP0 + 4 * channel, SRCP0 + 4 * channel + 2
        src, dst = self.rw(sp), self.rw(dp)
        if ctl & 0x100:
            self.wb(dst, self.rb(src))
            step = 1
        else:
            self.ww(dst, self.rw(src))
            step = 2
        if ctl & 0x600 == 0x200:
            self.ww(dp, dst + step)
        elif ctl & 0x600 == 0x400:
            self.ww(sp, src + step)
        self.pec_transfers += 1
        if count != 0xFF:
            self.ww(PECC0 + 2 * channel, (ctl & 0xFF00) | (count - 1))
            if count == 1:
                return False
        return True

    # ---- flags ----------------------------------------------------------------------
    def _add(self, a, b, m, s, carry, chain):
        r = a + b + carry
        f = self.psw & ~FLAGS
        if b == s:
            f |= E
        if r > m:
            f |= C
        r &= m
        if ~(a ^ b) & (a ^ r) & s:
            f |= V
        if not r and (not chain or self.psw & Z):
            f |= Z
        if r & s:
            f |= N
        self.psw = f
        return r

    def _sub(self, a, b, m, s, borrow, chain):
        r = a - b - borrow
        f = self.psw & ~FLAGS
        if b == s:
            f |= E
        if r < 0:
            f |= C
        r &= m
        if (a ^ b) & (a ^ r) & s:
            f |= V
        if not r and (not chain or self.psw & Z):
            f |= Z
        if r & s:
            f |= N
        self.psw = f
        return r

    def _logic(self, r, b, s):
        f = self.psw & ~FLAGS
        if b == s:
            f |= E
        if not r:
            f |= Z
        if r & s:
            f |= N
        self.psw = f
        return r

    def _moved(self, v, s):
        """The flags of a MOV: E, Z and N from the value, C and V as they were."""
        f = self.psw & ~(E | Z | N)
        if v == s:
            f |= E
        if not v:
            f |= Z
        if v & s:
            f |= N
        self.psw = f
        return v

    def _bit_flags(self, n, c, v, z):
        self.psw = (self.psw & ~FLAGS) | (N if n else 0) | (C if c else 0) | (V if v else 0) | (Z if z else 0)

    # ---- running ----------------------------------------------------------------------
    def run(self, max_insns, until=None):
        """Run until pc == until or the count is used up. Returns the instructions executed.

        IDLE stops the CPU until an interrupt; time then jumps to the next
        event a device has asked for, and the run ends early when there
        is none."""
        flash, low, kind, ops, hooks = self.flash, self.low, self.kind, self.ops, self.pc_hooks
        pages, mask, ext_ram = self.pages, self.flash_mask, self.ext_ram
        start = n = self.instructions
        end = start + max_insns
        while n < end:
            if n >= self.next_event:
                self.on_event()
                continue
            if self.idle:
                if self.attention:
                    self._service()              # trap() ends the idling
                if self.idle:
                    if self.next_event >= end:
                        break
                    n = self.instructions = self.next_event
                    continue
            limit = min(end, self.next_event)
            try:
                while n < limit:
                    if self.attention:
                        self._service()
                    ip = self.ip
                    pc = (self.csp << 16) | ip
                    if pc == until:
                        return n - start
                    if hooks:
                        hook = hooks.get(pc)
                        if hook is not None:
                            hook(self)
                            if self.ip != ip or self.idle:      # the hook moved the program on
                                continue
                    if pc >= 0x10000:
                        p = pages[pc >> 12]
                        if p == FLASH_PAGE:
                            o = pc & mask
                            op, b1, b2, b3 = flash[o], flash[o + 1], flash[o + 2], flash[o + 3]
                        elif p == RAM_PAGE:
                            op, b1, b2, b3 = ext_ram[pc], ext_ram[pc + 1], ext_ram[pc + 2], ext_ram[pc + 3]
                        else:
                            raise CpuError(f"executing where nothing is: 0x{pc:06X}")
                    elif kind[pc] == FLASH_KIND:
                        op, b1, b2, b3 = flash[pc], flash[pc + 1], flash[pc + 2], flash[pc + 3]
                    else:
                        op, b1, b2, b3 = low[pc], low[pc + 1], low[pc + 2], low[pc + 3]
                    self.at = pc
                    self.ip = (ip + LENGTH[op]) & 0xFFFF
                    ops[op](b1, b2, b3)
                    n += 1
                    self.instructions = n
                    if self.ext:
                        self.ext -= 1
                        if not self.ext:
                            self.ext_r = False
                            self.ext_mode = 0
                    if self.idle or self.next_event < limit:
                        break                        # IDLE, or a device moved its event up
            except _Fault as fault:
                if not self.hardware_traps:
                    raise
                if len(self.faults) < 1000:
                    self.faults.append((self.at, fault.flag, str(fault)))
                low[TFR] |= fault.flag
                self.csp, self.ip = self.at >> 16, self.at & 0xFFFF
                self.ext, self.ext_r, self.ext_mode = 0, False, 0
                self.trap(CLASS_B_TRAP)
                n = self.instructions = self.instructions + 1
        return n - start

    def step(self):
        return self.run(1)

    def _undefined(self, b1, b2, b3):
        self.csp, self.ip = self.at >> 16, self.at & 0xFFFF       # stay on it
        raise _Fault(UNDOPC, f"undefined opcode 0x{self.rb(self.at):02X} at 0x{self.at:06X}")

    # ---- the instruction set --------------------------------------------------------------
    def _build(self):
        ops = self.ops
        r, set_r, rbyte, set_rbyte = self.r, self.set_r, self.rbyte, self.set_rbyte
        rb, rw, wb, ww, da = self.rb, self.rw, self.wb, self.ww, self.da
        reg_rw, reg_ww, reg_rb, reg_wb = self.reg_rw, self.reg_ww, self.reg_rb, self.reg_wb
        add, sub, logic, moved = self._add, self._sub, self._logic, self._moved
        push, pop = self.push, self.pop
        cpu = self

        # ---- ADD ADDC SUB SUBC CMP XOR AND OR, in their ten forms ----
        kinds = {
            0x00: (lambda a, b, m, s: add(a, b, m, s, 0, False), True),
            0x10: (lambda a, b, m, s: add(a, b, m, s, (cpu.psw >> 1) & 1, True), True),
            0x20: (lambda a, b, m, s: sub(a, b, m, s, 0, False), True),
            0x30: (lambda a, b, m, s: sub(a, b, m, s, (cpu.psw >> 1) & 1, True), True),
            0x40: (lambda a, b, m, s: sub(a, b, m, s, 0, False), False),
            0x50: (lambda a, b, m, s: logic(a ^ b, b, s), True),
            0x60: (lambda a, b, m, s: logic(a & b, b, s), True),
            0x70: (lambda a, b, m, s: logic(a | b, b, s), True),
        }
        for base, (f, store) in kinds.items():
            def rw_rw(b1, b2, b3, f=f, store=store):
                n = b1 >> 4
                v = f(r(n), r(b1 & 15), 0xFFFF, 0x8000)
                if store:
                    set_r(n, v)

            def rb_rb(b1, b2, b3, f=f, store=store):
                n = b1 >> 4
                v = f(rbyte(n), rbyte(b1 & 15), 0xFF, 0x80)
                if store:
                    set_rbyte(n, v)

            def reg_mem_w(b1, b2, b3, f=f, store=store):
                v = f(reg_rw(b1), rw(da(b2 | (b3 << 8))), 0xFFFF, 0x8000)
                if store:
                    reg_ww(b1, v)

            def reg_mem_b(b1, b2, b3, f=f, store=store):
                v = f(reg_rb(b1), rb(da(b2 | (b3 << 8))), 0xFF, 0x80)
                if store:
                    reg_wb(b1, v)

            def mem_reg_w(b1, b2, b3, f=f):
                a = da(b2 | (b3 << 8))
                ww(a, f(rw(a), reg_rw(b1), 0xFFFF, 0x8000))

            def mem_reg_b(b1, b2, b3, f=f):
                a = da(b2 | (b3 << 8))
                wb(a, f(rb(a), reg_rb(b1), 0xFF, 0x80))

            def reg_imm_w(b1, b2, b3, f=f, store=store):
                v = f(reg_rw(b1), b2 | (b3 << 8), 0xFFFF, 0x8000)
                if store:
                    reg_ww(b1, v)

            def reg_imm_b(b1, b2, b3, f=f, store=store):
                v = f(reg_rb(b1), b2, 0xFF, 0x80)
                if store:
                    reg_wb(b1, v)

            def short_w(b1, b2, b3, f=f, store=store):
                n, x = b1 >> 4, b1 & 15
                if x & 8:
                    i = x & 3
                    p = r(i)
                    b = rw(da(p))
                    if x & 4:
                        set_r(i, (p + 2) & 0xFFFF)
                else:
                    b = x
                v = f(r(n), b, 0xFFFF, 0x8000)
                if store:
                    set_r(n, v)

            def short_b(b1, b2, b3, f=f, store=store):
                n, x = b1 >> 4, b1 & 15
                if x & 8:
                    i = x & 3
                    p = r(i)
                    b = rb(da(p))
                    if x & 4:
                        set_r(i, (p + 1) & 0xFFFF)
                else:
                    b = x
                v = f(rbyte(n), b, 0xFF, 0x80)
                if store:
                    set_rbyte(n, v)

            ops[base + 0], ops[base + 1] = rw_rw, rb_rb
            ops[base + 2], ops[base + 3] = reg_mem_w, reg_mem_b
            if store:                               # there is no CMP mem, reg
                ops[base + 4], ops[base + 5] = mem_reg_w, mem_reg_b
            ops[base + 6], ops[base + 7] = reg_imm_w, reg_imm_b
            ops[base + 8], ops[base + 9] = short_w, short_b

        # ---- CMPI1 CMPI2 CMPD1 CMPD2: compare, then step the register ----
        for base, step in ((0x80, 1), (0x90, 2), (0xA0, -1), (0xB0, -2)):
            def cmp_imm4(b1, b2, b3, step=step):
                n = b1 & 15
                a = r(n)
                sub(a, b1 >> 4, 0xFFFF, 0x8000, 0, False)
                set_r(n, a + step)

            def cmp_mem(b1, b2, b3, step=step):
                n = b1 & 15
                a = r(n)
                sub(a, rw(da(b2 | (b3 << 8))), 0xFFFF, 0x8000, 0, False)
                set_r(n, a + step)

            def cmp_imm16(b1, b2, b3, step=step):
                n = b1 & 15
                a = r(n)
                sub(a, b2 | (b3 << 8), 0xFFFF, 0x8000, 0, False)
                set_r(n, a + step)

            ops[base], ops[base + 2], ops[base + 6] = cmp_imm4, cmp_mem, cmp_imm16

        # ---- NEG CPL ----
        def neg(b1, b2, b3):
            n = b1 >> 4
            set_r(n, sub(0, r(n), 0xFFFF, 0x8000, 0, False))

        def negb(b1, b2, b3):
            n = b1 >> 4
            set_rbyte(n, sub(0, rbyte(n), 0xFF, 0x80, 0, False))

        def cpl(b1, b2, b3):
            n = b1 >> 4
            a = r(n)
            set_r(n, logic(a ^ 0xFFFF, a, 0x8000))

        def cplb(b1, b2, b3):
            n = b1 >> 4
            a = rbyte(n)
            set_rbyte(n, logic(a ^ 0xFF, a, 0x80))

        ops[0x81], ops[0xA1], ops[0x91], ops[0xB1] = neg, negb, cpl, cplb

        # ---- MUL DIV: results in MDL/MDH ----
        low = self.low

        def signed(v):
            return v - 0x10000 if v & 0x8000 else v

        def set_md(v):
            low[MDL], low[MDL + 1] = v & 0xFF, (v >> 8) & 0xFF
            low[MDH], low[MDH + 1] = (v >> 16) & 0xFF, (v >> 24) & 0xFF
            low[MDC] |= MDRIU

        def md_flags(value, overflow, negative):
            cpu.psw = (cpu.psw & ~FLAGS) | (Z if not value else 0) | (V if overflow else 0) | (N if negative else 0)

        def mul(b1, b2, b3):
            p = signed(r(b1 >> 4)) * signed(r(b1 & 15))
            set_md(p & 0xFFFFFFFF)
            md_flags(p, not -0x8000 <= p <= 0x7FFF, p < 0)

        def mulu(b1, b2, b3):
            p = r(b1 >> 4) * r(b1 & 15)
            set_md(p)
            md_flags(p, p > 0xFFFF, p & 0x80000000)

        def divide(dividend, divisor, is_signed):
            """Quotient to MDL, remainder to MDH; V and nothing else on a zero divisor or a quotient that does not fit."""
            if divisor == 0:
                cpu.psw = (cpu.psw & ~FLAGS) | V
                return
            q = abs(dividend) // abs(divisor)
            rem = abs(dividend) % abs(divisor)
            if (dividend < 0) != (divisor < 0):
                q = -q
            if dividend < 0:
                rem = -rem                              # the remainder takes the dividend's sign
            if not (-0x8000 <= q <= 0x7FFF if is_signed else q <= 0xFFFF):
                cpu.psw = (cpu.psw & ~FLAGS) | V
                return
            low[MDL], low[MDL + 1] = q & 0xFF, (q >> 8) & 0xFF
            low[MDH], low[MDH + 1] = rem & 0xFF, (rem >> 8) & 0xFF
            low[MDC] |= MDRIU
            md_flags(q & 0xFFFF, False, q & 0x8000)

        def mdl():
            return low[MDL] | (low[MDL + 1] << 8)

        def md32():
            return mdl() | (low[MDH] << 16) | (low[MDH + 1] << 24)

        def div(b1, b2, b3):
            divide(signed(mdl()), signed(r(b1 & 15)), True)

        def divu(b1, b2, b3):
            divide(mdl(), r(b1 & 15), False)

        def divl(b1, b2, b3):
            v = md32()
            divide(v - 0x100000000 if v & 0x80000000 else v, signed(r(b1 & 15)), True)

        def divlu(b1, b2, b3):
            divide(md32(), r(b1 & 15), False)

        ops[0x0B], ops[0x1B], ops[0x4B], ops[0x5B], ops[0x6B], ops[0x7B] = mul, mulu, div, divu, divl, divlu

        def prior(b1, b2, b3):
            v = r(b1 & 15)
            count = 16 - v.bit_length() if v else 0
            set_r(b1 >> 4, count)
            cpu.psw = (cpu.psw & ~FLAGS) | (Z if not v else 0)

        ops[0x2B] = prior

        # ---- shifts and rotates: C is the last bit out, V whether a 1 went out before it ----
        def shift_flags(result, last, lost):
            f = cpu.psw & ~FLAGS
            if last:
                f |= C
            if lost:
                f |= V
            if not result:
                f |= Z
            if result & 0x8000:
                f |= N
            cpu.psw = f

        def shl(n, count):
            a = r(n)
            v = (a << count) & 0xFFFF
            set_r(n, v)
            shift_flags(v, count and (a >> (16 - count)) & 1, False)

        def shr(n, count):
            a = r(n)
            v = a >> count
            set_r(n, v)
            shift_flags(v, count and (a >> (count - 1)) & 1, count > 1 and a & ((1 << (count - 1)) - 1))

        def ashr(n, count):
            a = r(n)
            v = (signed(a) >> count) & 0xFFFF
            set_r(n, v)
            shift_flags(v, count and (a >> (count - 1)) & 1, count > 1 and a & ((1 << (count - 1)) - 1))

        def rol(n, count):
            a = r(n)
            v = ((a << count) | (a >> (16 - count))) & 0xFFFF if count else a
            set_r(n, v)
            shift_flags(v, count and v & 1, False)

        def ror(n, count):
            a = r(n)
            v = ((a >> count) | (a << (16 - count))) & 0xFFFF if count else a
            set_r(n, v)
            shift_flags(v, count and v & 0x8000, count > 1 and a & ((1 << (count - 1)) - 1))

        for base, f in ((0x0C, rol), (0x2C, ror), (0x4C, shl), (0x6C, shr)):
            ops[base] = lambda b1, b2, b3, f=f: f(b1 >> 4, r(b1 & 15) & 15)
            ops[base + 0x10] = lambda b1, b2, b3, f=f: f(b1 & 15, b1 >> 4)
        ops[0xAC] = lambda b1, b2, b3: ashr(b1 >> 4, r(b1 & 15) & 15)
        ops[0xBC] = lambda b1, b2, b3: ashr(b1 & 15, b1 >> 4)

        # ---- MOV ----
        def mov_rw_rw(b1, b2, b3):
            set_r(b1 >> 4, moved(r(b1 & 15), 0x8000))

        def movb_rb_rb(b1, b2, b3):
            set_rbyte(b1 >> 4, moved(rbyte(b1 & 15), 0x80))

        def mov_rw_imm4(b1, b2, b3):
            set_r(b1 & 15, moved(b1 >> 4, 0x8000))

        def movb_rb_imm4(b1, b2, b3):
            set_rbyte(b1 & 15, moved(b1 >> 4, 0x80))

        def mov_reg_imm16(b1, b2, b3):
            reg_ww(b1, moved(b2 | (b3 << 8), 0x8000))

        def movb_reg_imm8(b1, b2, b3):
            reg_wb(b1, moved(b2, 0x80))

        def mov_reg_mem(b1, b2, b3):
            reg_ww(b1, moved(rw(da(b2 | (b3 << 8))), 0x8000))

        def movb_reg_mem(b1, b2, b3):
            reg_wb(b1, moved(rb(da(b2 | (b3 << 8))), 0x80))

        def mov_mem_reg(b1, b2, b3):
            ww(da(b2 | (b3 << 8)), moved(reg_rw(b1), 0x8000))

        def movb_mem_reg(b1, b2, b3):
            wb(da(b2 | (b3 << 8)), moved(reg_rb(b1), 0x80))

        def mov_rw_ind(b1, b2, b3):                     # MOV Rwn, [Rwm]
            set_r(b1 >> 4, moved(rw(da(r(b1 & 15))), 0x8000))

        def movb_rb_ind(b1, b2, b3):
            set_rbyte(b1 >> 4, moved(rb(da(r(b1 & 15))), 0x80))

        def mov_ind_rw(b1, b2, b3):                     # MOV [Rwm], Rwn
            ww(da(r(b1 & 15)), moved(r(b1 >> 4), 0x8000))

        def movb_ind_rb(b1, b2, b3):
            wb(da(r(b1 & 15)), moved(rbyte(b1 >> 4), 0x80))

        def mov_rw_ind_inc(b1, b2, b3):                 # MOV Rwn, [Rwm+]
            m = b1 & 15
            p = r(m)
            v = rw(da(p))
            set_r(m, (p + 2) & 0xFFFF)
            set_r(b1 >> 4, moved(v, 0x8000))

        def movb_rb_ind_inc(b1, b2, b3):
            m = b1 & 15
            p = r(m)
            v = rb(da(p))
            set_r(m, (p + 1) & 0xFFFF)
            set_rbyte(b1 >> 4, moved(v, 0x80))

        def mov_dec_ind_rw(b1, b2, b3):                 # MOV [-Rwm], Rwn
            m = b1 & 15
            v = r(b1 >> 4)
            p = (r(m) - 2) & 0xFFFF
            set_r(m, p)
            ww(da(p), moved(v, 0x8000))

        def movb_dec_ind_rb(b1, b2, b3):
            m = b1 & 15
            v = rbyte(b1 >> 4)
            p = (r(m) - 1) & 0xFFFF
            set_r(m, p)
            wb(da(p), moved(v, 0x80))

        def mov_ind_ind(b1, b2, b3):                    # MOV [Rwn], [Rwm]
            ww(da(r(b1 >> 4)), moved(rw(da(r(b1 & 15))), 0x8000))

        def movb_ind_ind(b1, b2, b3):
            wb(da(r(b1 >> 4)), moved(rb(da(r(b1 & 15))), 0x80))

        def mov_ind_inc_ind(b1, b2, b3):                # MOV [Rwn+], [Rwm]
            n = b1 >> 4
            p = r(n)
            ww(da(p), moved(rw(da(r(b1 & 15))), 0x8000))
            set_r(n, (p + 2) & 0xFFFF)

        def movb_ind_inc_ind(b1, b2, b3):
            n = b1 >> 4
            p = r(n)
            wb(da(p), moved(rb(da(r(b1 & 15))), 0x80))
            set_r(n, (p + 1) & 0xFFFF)

        def mov_ind_ind_inc(b1, b2, b3):                # MOV [Rwn], [Rwm+]
            m = b1 & 15
            p = r(m)
            ww(da(r(b1 >> 4)), moved(rw(da(p)), 0x8000))
            set_r(m, (p + 2) & 0xFFFF)

        def movb_ind_ind_inc(b1, b2, b3):
            m = b1 & 15
            p = r(m)
            wb(da(r(b1 >> 4)), moved(rb(da(p)), 0x80))
            set_r(m, (p + 1) & 0xFFFF)

        def mov_rw_idx(b1, b2, b3):                     # MOV Rwn, [Rwm + #data16]
            set_r(b1 >> 4, moved(rw(da((r(b1 & 15) + (b2 | (b3 << 8))) & 0xFFFF)), 0x8000))

        def movb_rb_idx(b1, b2, b3):
            set_rbyte(b1 >> 4, moved(rb(da((r(b1 & 15) + (b2 | (b3 << 8))) & 0xFFFF)), 0x80))

        def mov_idx_rw(b1, b2, b3):                     # MOV [Rwm + #data16], Rwn
            ww(da((r(b1 & 15) + (b2 | (b3 << 8))) & 0xFFFF), moved(r(b1 >> 4), 0x8000))

        def movb_idx_rb(b1, b2, b3):
            wb(da((r(b1 & 15) + (b2 | (b3 << 8))) & 0xFFFF), moved(rbyte(b1 >> 4), 0x80))

        def mov_ind_mem(b1, b2, b3):                    # MOV [Rwn], mem
            ww(da(r(b1 & 15)), moved(rw(da(b2 | (b3 << 8))), 0x8000))

        def movb_ind_mem(b1, b2, b3):
            wb(da(r(b1 & 15)), moved(rb(da(b2 | (b3 << 8))), 0x80))

        def mov_mem_ind(b1, b2, b3):                    # MOV mem, [Rwn]
            ww(da(b2 | (b3 << 8)), moved(rw(da(r(b1 & 15))), 0x8000))

        def movb_mem_ind(b1, b2, b3):
            wb(da(b2 | (b3 << 8)), moved(rb(da(r(b1 & 15))), 0x80))

        ops[0xF0], ops[0xF1], ops[0xE0], ops[0xE1] = mov_rw_rw, movb_rb_rb, mov_rw_imm4, movb_rb_imm4
        ops[0xE6], ops[0xE7] = mov_reg_imm16, movb_reg_imm8
        ops[0xF2], ops[0xF3], ops[0xF6], ops[0xF7] = mov_reg_mem, movb_reg_mem, mov_mem_reg, movb_mem_reg
        ops[0xA8], ops[0xA9], ops[0xB8], ops[0xB9] = mov_rw_ind, movb_rb_ind, mov_ind_rw, movb_ind_rb
        ops[0x98], ops[0x99], ops[0x88], ops[0x89] = mov_rw_ind_inc, movb_rb_ind_inc, mov_dec_ind_rw, movb_dec_ind_rb
        ops[0xC8], ops[0xC9], ops[0xD8], ops[0xD9] = mov_ind_ind, movb_ind_ind, mov_ind_inc_ind, movb_ind_inc_ind
        ops[0xE8], ops[0xE9] = mov_ind_ind_inc, movb_ind_ind_inc
        ops[0xD4], ops[0xF4], ops[0xC4], ops[0xE4] = mov_rw_idx, movb_rb_idx, mov_idx_rw, movb_idx_rb
        ops[0x84], ops[0xA4], ops[0x94], ops[0xB4] = mov_ind_mem, movb_ind_mem, mov_mem_ind, movb_mem_ind

        # ---- MOVBZ MOVBS: a byte into a word ----
        def extended(v, sign):
            if sign and v & 0x80:
                v |= 0xFF00
            cpu.psw = (cpu.psw & ~(E | Z | N)) | (Z if not v else 0) | (N if v & 0x8000 else 0)
            return v

        for base, sign in ((0xC0, False), (0xD0, True)):
            ops[base] = lambda b1, b2, b3, sign=sign: set_r(b1 & 15, extended(rbyte(b1 >> 4), sign))
            ops[base + 2] = lambda b1, b2, b3, sign=sign: reg_ww(b1, extended(rb(da(b2 | (b3 << 8))), sign))
            ops[base + 5] = lambda b1, b2, b3, sign=sign: ww(da(b2 | (b3 << 8)), extended(reg_rb(b1), sign))

        # ---- bits ----
        bit_addr = self._bit_addr
        bit_flags = self._bit_flags

        def bfldl(b1, b2, b3):                          # 0A QQ @@ ##
            a = bit_addr(b1)
            v = (rw(a) & ~b2 & 0xFFFF) | b3
            ww(a, v)
            logic(v, 0, 0x8000)

        def bfldh(b1, b2, b3):                          # 1A QQ ## @@
            a = bit_addr(b1)
            v = (rw(a) & ~(b3 << 8) & 0xFFFF) | (b2 << 8)
            ww(a, v)
            logic(v, 0, 0x8000)

        ops[0x0A], ops[0x1A] = bfldl, bfldh

        for q in range(16):
            def bclr(b1, b2, b3, mask=1 << q):
                a = bit_addr(b1)
                v = rw(a)
                ww(a, v & ~mask)
                bit_flags(v & mask, 0, 0, not v & mask)

            def bset(b1, b2, b3, mask=1 << q):
                a = bit_addr(b1)
                v = rw(a)
                ww(a, v | mask)
                bit_flags(v & mask, 0, 0, not v & mask)

            ops[(q << 4) | 0xE], ops[(q << 4) | 0xF] = bclr, bset

        def bit_pair(b1, b2, b3):
            """op QQ ZZ qz: the source bit Q.q, the destination's address, mask and present value."""
            src = (rw(bit_addr(b1)) >> (b3 >> 4)) & 1
            a = bit_addr(b2)
            mask = 1 << (b3 & 15)
            return src, a, mask, rw(a)

        def bmov(b1, b2, b3):
            src, a, mask, v = bit_pair(b1, b2, b3)
            ww(a, v | mask if src else v & ~mask)
            bit_flags(src, 0, 0, not src)

        def bmovn(b1, b2, b3):
            src, a, mask, v = bit_pair(b1, b2, b3)
            ww(a, v & ~mask if src else v | mask)
            bit_flags(src, 0, 0, not src)

        def bit_logic(fn, store=True):
            def op(b1, b2, b3):
                src, a, mask, v = bit_pair(b1, b2, b3)
                dst = 1 if v & mask else 0
                if store:
                    ww(a, v | mask if fn(dst, src) else v & ~mask)
                bit_flags(dst ^ src, dst & src, dst | src, not (dst | src))
            return op

        ops[0x4A], ops[0x3A] = bmov, bmovn
        ops[0x2A] = bit_logic(None, store=False)        # BCMP
        ops[0x5A] = bit_logic(lambda d, s: d | s)
        ops[0x6A] = bit_logic(lambda d, s: d & s)
        ops[0x7A] = bit_logic(lambda d, s: d ^ s)

        def rel(offset):
            cpu.ip = (cpu.ip + 2 * (offset - 256 if offset & 0x80 else offset)) & 0xFFFF

        def jb(b1, b2, b3):                             # 8A QQ rr q0
            if (rw(bit_addr(b1)) >> (b3 >> 4)) & 1:
                rel(b2)

        def jnb(b1, b2, b3):
            if not (rw(bit_addr(b1)) >> (b3 >> 4)) & 1:
                rel(b2)

        def jbc(b1, b2, b3):
            a = bit_addr(b1)
            v = rw(a)
            mask = 1 << (b3 >> 4)
            bit_flags(v & mask, 0, 0, not v & mask)
            if v & mask:
                ww(a, v & ~mask)
                rel(b2)

        def jnbs(b1, b2, b3):
            a = bit_addr(b1)
            v = rw(a)
            mask = 1 << (b3 >> 4)
            bit_flags(v & mask, 0, 0, not v & mask)
            if not v & mask:
                ww(a, v | mask)
                rel(b2)

        ops[0x8A], ops[0x9A], ops[0xAA], ops[0xBA] = jb, jnb, jbc, jnbs

        # ---- jumps and calls ----
        for cc in range(16):
            def jmpr(b1, b2, b3, row=COND[cc]):
                if row[cpu.psw & 0x1F]:
                    cpu.ip = (cpu.ip + 2 * (b1 - 256 if b1 & 0x80 else b1)) & 0xFFFF

            ops[(cc << 4) | 0xD] = jmpr

        def jmpa(b1, b2, b3):
            if COND[b1 >> 4][cpu.psw & 0x1F]:
                cpu.ip = b2 | (b3 << 8)

        def jmpi(b1, b2, b3):
            if COND[b1 >> 4][cpu.psw & 0x1F]:
                cpu.ip = r(b1 & 15)

        def jmps(b1, b2, b3):
            cpu.csp, cpu.ip = b1, b2 | (b3 << 8)

        def calla(b1, b2, b3):
            if COND[b1 >> 4][cpu.psw & 0x1F]:
                push(cpu.ip)
                cpu.ip = b2 | (b3 << 8)

        def calli(b1, b2, b3):
            if COND[b1 >> 4][cpu.psw & 0x1F]:
                push(cpu.ip)
                cpu.ip = r(b1 & 15)

        def callr(b1, b2, b3):
            push(cpu.ip)
            rel(b1)

        def calls(b1, b2, b3):
            push(cpu.csp)
            push(cpu.ip)
            cpu.csp, cpu.ip = b1, b2 | (b3 << 8)

        def pcall(b1, b2, b3):
            push(moved(reg_rw(b1), 0x8000))
            push(cpu.ip)
            cpu.ip = b2 | (b3 << 8)

        def trap(b1, b2, b3):
            cpu.trap(b1 >> 1)

        def ret(b1, b2, b3):
            cpu.ip = pop()

        def rets(b1, b2, b3):
            cpu.ip = pop()
            cpu.csp = pop() & 0xFF

        def retp(b1, b2, b3):
            cpu.ip = pop()
            reg_ww(b1, moved(pop(), 0x8000))

        def reti(b1, b2, b3):
            cpu.ip = pop()
            if not rw(SYSCON) & SGTDIS:
                cpu.csp = pop() & 0xFF
            cpu.psw = pop()
            cpu.attention = True

        ops[0xEA], ops[0x9C], ops[0xFA] = jmpa, jmpi, jmps
        ops[0xCA], ops[0xAB], ops[0xBB], ops[0xDA], ops[0xE2], ops[0x9B] = calla, calli, callr, calls, pcall, trap
        ops[0xCB], ops[0xDB], ops[0xEB], ops[0xFB] = ret, rets, retp, reti

        # ---- stack, context ----
        def push_reg(b1, b2, b3):
            push(moved(reg_rw(b1), 0x8000))

        def pop_reg(b1, b2, b3):
            reg_ww(b1, moved(pop(), 0x8000))

        def scxt_imm(b1, b2, b3):
            push(reg_rw(b1))
            reg_ww(b1, b2 | (b3 << 8))

        def scxt_mem(b1, b2, b3):
            v = rw(da(b2 | (b3 << 8)))
            push(reg_rw(b1))
            reg_ww(b1, v)

        ops[0xEC], ops[0xFC], ops[0xC6], ops[0xD6] = push_reg, pop_reg, scxt_imm, scxt_mem

        # ---- ATOMIC and the EXT sequences: the count includes this instruction ----
        def atomic_extr(b1, b2, b3):
            cpu.ext = ((b1 >> 4) & 3) + 2
            cpu.ext_r = bool(b1 & 0x80)
            cpu.ext_mode = 0

        def ext_imm(b1, b2, b3):                        # D7: EXTS #seg, EXTP #pag, and their R forms
            cpu.ext = ((b1 >> 4) & 3) + 2
            cpu.ext_r = bool(b1 & 0x80)
            if b1 & 0x40:
                cpu.ext_mode, cpu.ext_base = 1, ((b2 | (b3 << 8)) & 0x3FF) << 14
            else:
                cpu.ext_mode, cpu.ext_base = 2, b2 << 16

        def ext_reg(b1, b2, b3):                        # DC: the same with the page or segment in a register
            v = r(b1 & 15)
            cpu.ext = ((b1 >> 4) & 3) + 2
            cpu.ext_r = bool(b1 & 0x80)
            if b1 & 0x40:
                cpu.ext_mode, cpu.ext_base = 1, (v & 0x3FF) << 14
            else:
                cpu.ext_mode, cpu.ext_base = 2, (v & 0xFF) << 16

        ops[0xD1], ops[0xD7], ops[0xDC] = atomic_extr, ext_imm, ext_reg

        # ---- system ----
        def nop(b1, b2, b3):
            pass

        def srvwdt(b1, b2, b3):
            cpu.watchdog_services += 1

        def diswdt(b1, b2, b3):
            cpu.watchdog_disabled = True

        def einit(b1, b2, b3):
            cpu.einit = True

        def idle(b1, b2, b3):
            cpu.idle = True

        def srst(b1, b2, b3):
            cpu.resets += 1
            cpu.reset()

        def pwrdn(b1, b2, b3):
            cpu.idle = True

        ops[0xCC], ops[0xA7], ops[0xA5], ops[0xB5], ops[0x87], ops[0xB7], ops[0x97] = nop, srvwdt, diswdt, einit, idle, srst, pwrdn
