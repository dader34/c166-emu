"""The shared peripherals on a bare core: the CAPCOM timers, the CAN transmit rule, the
PWM unit, the A/D converter's source hook and the external-RAM write hook."""
from c166 import sfr
from c166.cpu import Cpu, NEVER
from c166.periph import CapcomTimers, Can, Capin, Timer, Pwm, Adc


class Bench:
    """The least of a board: a CPU, a clock and the devices' event plan."""

    def __init__(self, fcpu=24_000_000, ips=12_000_000):
        self.cpu = Cpu()
        self.fcpu, self.ips = fcpu, ips
        self.unsupported = set()
        self.devices = []
        self.cpu.on_event = self._event

    def read16(self, a):
        return self.cpu.rw(a)

    def plan(self):
        self.cpu.next_event = min((d.next_at for d in self.devices), default=NEVER)

    def _event(self):
        for d in self.devices:
            if self.cpu.instructions >= d.next_at:
                d.service(self.cpu.instructions)
        self.plan()

    def idle(self, n):
        """Run n instructions of NOPs from the external RAM at 0xC000."""
        self.cpu.write(0xC000, b"\xcc\x00" * 0x800)
        while n > 0:
            self.cpu.csp, self.cpu.ip = 0, 0xC000
            n -= self.cpu.run(min(n, 0x7FF))


def test_t1_counts_the_clock_and_reloads():
    b = Bench()
    t = CapcomTimers(b)
    b.devices.append(t)
    b.cpu.ww(sfr.T1REL, 0xFF00)
    b.cpu.ww(sfr.T1, 0xFFF0)
    b.cpu.ww(sfr.T01CON, 0x4000)                 # T1 running, fCPU/8: 3 MHz, 4 instructions a count
    b.idle(40)
    assert b.cpu.rw(sfr.T1) == 0xFFFA and b.cpu.rw(sfr.T0) == 0
    b.idle(40)                                   # past the overflow at 16 counts
    assert t.timers[1].wraps == 1 and b.cpu.low[sfr.T1IC] & 0x80
    assert b.cpu.rw(sfr.T1) == 0xFF00 + 4


def test_t0_prescaler_and_counter_mode():
    b = Bench()
    t = CapcomTimers(b)
    b.devices.append(t)
    b.cpu.ww(sfr.T01CON, 0x0042)                 # T0 running, fCPU/32
    b.idle(160)
    assert b.cpu.rw(sfr.T0) == 10
    b.cpu.ww(sfr.T0, 0)
    b.cpu.ww(sfr.T01CON, 0x0048)                 # counter mode, input x00: T6's wraps
    b.idle(200)
    assert b.cpu.rw(sfr.T0) == 0                 # the clock no longer counts
    t.t6_wrapped()
    t.t6_wrapped()
    assert b.cpu.rw(sfr.T0) == 2
    t.count_edge(1)                              # x00 takes no pin edges
    assert b.cpu.rw(sfr.T0) == 2


def test_counter_mode_counts_pin_edges_reloads_and_matches():
    b = Bench()
    t = CapcomTimers(b, second_unit=True)        # T7 counting the crank on T7IN
    b.devices.append(t)
    hits = []
    t.on_compare = lambda n, mode: hits.append(n)
    b.cpu.ww(sfr.T7REL, 0xFFFC)
    b.cpu.ww(sfr.T7, 0xFFFD)
    b.cpu.ww(sfr.CC24, 0xFFFE)
    b.cpu.ww(sfr.CCM6, 0x0005)                   # CC24: compare mode 1 on T7
    b.cpu.ww(sfr.T78CON, 0x004A)                 # T7 counting falling edges
    t.count_edge(1)                              # rising: not counted
    t.count_edge(0)
    assert b.cpu.rw(sfr.T7) == 0xFFFE and hits == [24] and b.cpu.low[sfr.CC24IC] & 0x80
    t.count_edge(0)
    t.count_edge(0)                              # past 0xFFFF: the reload
    assert b.cpu.rw(sfr.T7) == 0xFFFC and t.timers[0].wraps == 1 and b.cpu.low[sfr.T7IC] & 0x80
    b.cpu.ww(sfr.T7, 0xFFFE)                     # written onto the compare value: a match
    assert hits == [24, 24]


def test_compare_requests_its_interrupt_each_time_round():
    b = Bench()
    t = CapcomTimers(b, second_unit=True)
    b.devices.append(t)
    hits = []
    t.on_compare = lambda n, mode: hits.append((n, b.cpu.instructions))
    b.cpu.ww(sfr.CC26, 0x0010)
    b.cpu.ww(sfr.CCM6, 0x0D00)                   # CC26: compare mode 1 on T8
    b.cpu.ww(sfr.T8REL, 0xFF00)
    b.cpu.ww(sfr.T8, 0x0000)
    b.cpu.ww(sfr.T78CON, 0x4000)                 # T8 running, 4 instructions a count
    b.idle(80)
    assert hits and hits[0][0] == 26 and b.cpu.low[sfr.CC26IC] & 0x80
    assert hits[0][1] in range(64, 70)           # 16 counts
    b.cpu.ww(sfr.CC26, 0xFF80)                   # matches once a period from now on: 256 counts
    b.idle(4 * 0x10000)
    first = len(hits)
    b.idle(4 * 256 * 4)
    assert len(hits) - first == 4


def test_capture_latches_its_timer_on_the_edges_its_mode_takes():
    b = Bench()
    t = CapcomTimers(b)
    b.devices.append(t)
    b.cpu.ww(sfr.T01CON, 0x4000)                 # T1 running, 4 instructions a count
    b.cpu.ww(sfr.CCM3, 0xAB00)                   # CC14 both edges on T1, CC15 falling on T1
    b.cpu.low[sfr.CC15IC] = 0x40                 # enabled
    b.idle(40)
    t.edge(15, 0)                                # falling: CC15 takes T1
    assert b.cpu.rw(sfr.CC15) == 10 and b.cpu.low[sfr.CC15IC] & 0x80
    b.cpu.low[sfr.CC15IC] = 0x40
    b.idle(40)
    t.edge(15, 1)                                # rising: not CC15's edge
    assert b.cpu.rw(sfr.CC15) == 10 and not b.cpu.low[sfr.CC15IC] & 0x80
    t.edge(14, 0)
    b.idle(20)
    t.edge(14, 1)                                # both edges: the rising one too
    assert b.cpu.rw(sfr.CC14) == 25 and t.captures == 3
    t.edge(14, 1)                                # no change, no capture
    assert t.captures == 3


def test_the_pins_read_back_in_p2_on_input_bits():
    b = Bench()
    t = CapcomTimers(b)
    b.devices.append(t)
    b.cpu.ww(sfr.DP2, 0x00FF)                    # P2.0-7 outputs, P2.8-15 inputs
    b.cpu.ww(sfr.P2, 0x0081)
    t.edge(14, 0)
    t.edge(13, 1)
    p2 = b.cpu.rw(sfr.P2)
    assert p2 & 0x00FF == 0x81                   # the latch on the outputs
    assert not p2 & 0x4000 and p2 & 0x2000       # the driven levels on the inputs


def test_a_capture_at_pec_level_is_a_transfer():
    """The MS43 takes its crank teeth this way: CC15 at level 15 group 3 is
    PEC channel 7, which copies each captured time into a buffer."""
    b = Bench()
    t = CapcomTimers(b)
    b.devices.append(t)
    b.cpu.ww(sfr.T01CON, 0x4000)
    b.cpu.ww(sfr.CCM3, 0xA000)                   # CC15 falling edges on T1
    b.cpu.ww(sfr.SRCP7, sfr.CC15)
    b.cpu.ww(sfr.DSTP7, 0xE5B4)
    b.cpu.ww(sfr.PECC7, 0x0212)                  # words, destination ++, 18 transfers
    b.cpu.low[sfr.CC15IC] = 0x7F                 # IE, level 15, group 3
    b.cpu.psw |= 0x0800                          # IEN
    for k in range(3):
        b.idle(40)
        t.edge(15, 0)
        b.idle(2)
        t.edge(15, 1)
    assert b.cpu.pec_transfers == 3 and b.cpu.rw(sfr.DSTP7) == 0xE5B4 + 6
    first, second = b.cpu.rw(0xE5B4), b.cpu.rw(0xE5B6)
    assert second - first in (10, 11)            # 42 instructions apart, 4 a count


def test_capin_captures_t5_into_caprel_and_requests_cric():
    b = Bench()
    t5 = Timer(b, "T5", 4)
    b.devices.append(t5)
    cap = Capin(b, t5)
    b.cpu.ww(sfr.T5CON, 0xA040)                  # running, capture on falling CAPIN edges
    b.cpu.low[sfr.CRIC] = 0x40
    b.idle(100)
    cap.edge(0)
    got = b.cpu.rw(sfr.CAPREL)
    assert got == t5.now() and got > 0 and b.cpu.low[sfr.CRIC] & 0x80
    b.cpu.low[sfr.CRIC] = 0x40
    cap.edge(1)                                  # rising: not this mode's edge
    assert not b.cpu.low[sfr.CRIC] & 0x80
    b.cpu.ww(sfr.T5CON, 0xE040)                  # ...and T5CLR
    b.idle(40)
    cap.edge(0)
    assert t5.now() == 0 and cap.captures == 2


def object_at(n):
    return 0xEF00 + 0x10 * n


def setup_tx(b, n, ident, data):
    a = object_at(n)
    w = (ident << 5) & 0xFFFF
    b.cpu.ww(a + 2, ((w >> 8) | (w << 8)) & 0xFFFF)
    b.cpu.low[a + 6] = (len(data) << 4) | 0x08
    b.cpu.low[a + 8:a + 8 + len(data)] = data


def test_txrq_with_cpuupd_reset_sends_once():
    b = Bench()
    can = Can(b)
    b.devices.append(can)
    setup_tx(b, 14, 0x316, b"\x01\x02")
    b.cpu.ww(object_at(14), 0x6595)              # the MS43's: TXRQ set, CPUUPD reset in one write
    b.idle(10_000)
    assert can.sent == [(0x316, b"\x01\x02")]
    assert b.cpu.rw(object_at(14)) & 0x3000 == 0x1000          # TXRQ reads reset again


def test_gs20_sequence_sends_at_txrq_only():
    b = Bench()
    can = Can(b)
    b.devices.append(can)
    setup_tx(b, 6, 0x43F, b"\xaa")
    for v in (0xFAFF, 0xF7FF, 0xEFFF):           # CPUUPD set, CPUUPD reset, TXRQ set
        b.cpu.ww(object_at(6), v)
    b.idle(10_000)
    assert can.sent == [(0x43F, b"\xaa")]
    b.cpu.ww(object_at(6), 0xEAFF)               # TXRQ with CPUUPD set: the CPU still owns the object
    b.idle(10_000)
    assert len(can.sent) == 1


def test_pwm_period_interrupt_and_counter():
    b = Bench()
    pwm = Pwm(b)
    b.devices.append(pwm)
    b.cpu.ww(sfr.PP2, 999)                       # 1000 counts at fCPU: 500 instructions
    b.cpu.ww(sfr.PW2, 250)
    b.cpu.ww(sfr.PWMCON0, 0x0404)                # PT2 running at fCPU, PIE2
    b.plan()
    b.idle(300)
    assert 590 <= b.cpu.rw(sfr.PT2) <= 610 and not b.cpu.low[sfr.PWMIC] & 0x80
    b.idle(300)                                  # past the period's end
    assert pwm.periods[2] == 1 and b.cpu.low[sfr.PWMIC] & 0x80
    assert b.cpu.rw(sfr.PWMCON0) & 0x4000        # PIR2
    b.idle(5000)
    assert pwm.periods[2] == 11
    assert pwm.duty(2) == 0.75


def test_pwm_slow_clock_and_centre_aligned():
    b = Bench()
    pwm = Pwm(b)
    b.devices.append(pwm)
    b.cpu.ww(sfr.PP0, 99)
    b.cpu.ww(sfr.PWMCON1, 0x0010)                # channel 0 centre-aligned: up 100, down 100
    b.cpu.ww(sfr.PWMCON0, 0x0111)                # PT0 at fCPU/64, PIE0
    b.plan()
    b.idle(200 * 32 + 50)                        # one period: 200 counts, 32 instructions each
    assert pwm.periods[0] == 1


def test_adc_source_answers_first():
    b = Bench()
    adc = Adc(b)
    b.devices.append(adc)
    adc.channels = {3: 0x111}
    adc.source = lambda ch: 0x222 if ch == 3 and b.cpu.low[0xF000] else None
    b.cpu.ww(sfr.ADCON, 0x0083)                  # one conversion of channel 3
    b.plan()
    b.idle(2000)
    assert b.cpu.rw(sfr.ADDAT) == 0x3111         # the source has nothing: the channel's value
    b.cpu.low[0xF000] = 1
    b.cpu.ww(sfr.ADCON, 0x0083)
    b.plan()
    b.idle(2000)
    assert b.cpu.rw(sfr.ADDAT) == 0x3222


def test_external_write_hook_sees_writes_and_keeps_the_ram():
    b = Bench()
    b.cpu.map_ram(0x40000, 0x1000)
    seen = []
    b.cpu.hook_ext_write(0x40100, lambda a, v, size: seen.append((a, v, size)))
    b.cpu.ww(0x40100, 0x1234)
    b.cpu.wb(0x40100, 0x56)
    b.cpu.ww(0x40102, 0xABCD)                    # the page's other addresses: plain RAM
    assert seen == [(0x40100, 0x1234, 2), (0x40100, 0x56, 1)]
    assert b.cpu.rw(0x40100) == 0x1256 and b.cpu.rw(0x40102) == 0xABCD
