"""The CPU core on hand-assembled instructions: no image needed. Code is
put in the external RAM at 0x00C000 and run from there."""
import pytest

from c166.cpu import Cpu, CpuError, N, C, V, Z, E, MDL, MDH

CODE = 0xC000


def run(code, cpu=None, steps=1000):
    cpu = cpu or Cpu()
    cpu.write(CODE, bytes(code))
    cpu.csp, cpu.ip = 0, CODE
    cpu.run(steps, until=CODE + len(code))
    assert cpu.pc == CODE + len(code), f"stopped at 0x{cpu.pc:06X}"
    return cpu


def flags(cpu):
    return cpu.psw & (N | C | V | Z | E)


def mov(n, value):
    """MOV Rn, #data16"""
    return [0xE6, 0xF0 | n, value & 0xFF, value >> 8]


def test_add_overflow_and_carry():
    cpu = run(mov(1, 0x7FFF) + mov(2, 1) + [0x00, 0x12])               # ADD R1, R2
    assert cpu.r(1) == 0x8000 and flags(cpu) == V | N
    cpu = run(mov(1, 0xFFFF) + mov(2, 1) + [0x00, 0x12])
    assert cpu.r(1) == 0 and flags(cpu) == C | Z


def test_addc_carries_and_chains_zero():
    low = mov(1, 0xFFFF) + mov(2, 1) + [0x00, 0x12]                      # leaves C and Z
    cpu = run(low + mov(3, 0) + mov(4, 0) + [0x10, 0x34])               # ADDC R3, R4
    assert cpu.r(3) == 1 and flags(cpu) == 0
    cpu = run(low + mov(3, 0xFFFF) + mov(4, 0) + [0x10, 0x34])
    assert cpu.r(3) == 0 and flags(cpu) == C | Z                         # zero only because the low word was


def test_compare_and_conditions():
    cpu = run(mov(1, 1) + mov(2, 2) + [0x40, 0x12])                     # CMP R1, R2
    assert cpu.r(1) == 1 and flags(cpu) == C | N
    # -1 < 1 signed, but not unsigned: JMPR cc_SLT skips the MOV, JMPR cc_ULT does not
    cpu = run(mov(1, 0xFFFF) + mov(2, 1) + mov(3, 0) + [0x40, 0x12, 0xCD, 0x01, 0xE0, 0x13, 0x8D, 0x01, 0xE0, 0x24])
    assert cpu.r(3) == 0 and cpu.r(4) == 2


def test_mov_flags():
    cpu = run(mov(1, 0x8000))
    assert flags(cpu) == E | N
    cpu = run(mov(1, 0xFFFF) + mov(2, 1) + [0x00, 0x12] + mov(3, 5))    # C survives a MOV
    assert flags(cpu) == C


def test_byte_operations():
    cpu = run([0xE7, 0xF2, 0x80, 0x00, 0x01, 0x22])                     # MOVB RL1, #0x80; ADDB RL1, RL1
    assert cpu.rbyte(2) == 0 and flags(cpu) == C | Z | V | E
    cpu = run([0xE7, 0xF2, 0x80, 0x00, 0xD0, 0x22, 0xC0, 0x23])         # MOVBS R2, RL1; MOVBZ R3, RL1
    assert cpu.r(2) == 0xFF80 and cpu.r(3) == 0x0080


def test_data_pages():
    cpu = run([0xE6, 0x02, 0x03, 0x00] + mov(1, 0x1234) + [0xF6, 0xF1, 0x00, 0x81])     # MOV DPP2, #3; MOV 0x8100, R1
    assert cpu.rw(0x00C100) == 0x1234
    assert cpu.rw(0xFE04) == 3


def test_extended_segment_and_page():
    for prefix in ([0xD7, 0x00, 0x0C, 0x00], [0xD7, 0x40, 0x30, 0x00]):  # EXTS #0x0C, #1 / EXTP #0x30, #1
        cpu = Cpu()
        cpu.flash[0x100:0x102] = b"\x11\x11"
        cpu.flash[0x40100:0x40102] = b"\x34\x12"                         # 0x0C0100
        run(prefix + [0xF2, 0xF1, 0x00, 0x01, 0xF2, 0xF2, 0x00, 0x01], cpu)   # MOV R1, 0x0100; MOV R2, 0x0100
        assert cpu.r(1) == 0x1234 and cpu.r(2) == 0x1111                  # the override lasts one instruction


def test_extended_registers():
    cpu = run([0xD1, 0x80, 0xE6, 0x10, 0x34, 0x12, 0xE6, 0x10, 0x78, 0x56])   # EXTR #1; MOV reg 0x10, ..; MOV reg 0x10, ..
    assert cpu.rw(0xF020) == 0x1234 and cpu.rw(0xFE20) == 0x5678


def test_a_byte_written_to_an_sfr_clears_its_other_byte():
    cpu = run([0xE6, 0x10, 0x34, 0x12, 0xE7, 0x10, 0x56, 0x00])
    assert cpu.rw(0xFE20) == 0x0056


def test_stack_and_calls():
    code = mov(1, 0x1111) + [0xEC, 0xF1, 0xFC, 0xF2]                     # PUSH R1; POP R2
    code += [0xCA, 0x00, 0x20, 0xC0]                                     # CALLA cc_UC, 0xC020
    code += [0xDA, 0x00, 0x30, 0xC0]                                     # CALLS 0, 0xC030
    code += [0xBB, 0x00]                                                 # CALLR +0: pushes the next address
    cpu = Cpu()
    cpu.write(0xC020, bytes([0xE0, 0x73, 0xCB, 0x00]))                   # MOV R3, #7; RET
    cpu.write(0xC030, bytes([0xE0, 0x94, 0xDB, 0x00]))                   # MOV R4, #9; RETS
    sp = cpu.sp
    run(code, cpu)
    assert (cpu.r(2), cpu.r(3), cpu.r(4)) == (0x1111, 7, 9)
    assert cpu.sp == sp - 2 and cpu.rw(cpu.sp) == CODE + len(code)


def test_indirect_jump_and_loop():
    # MOV R1, #0; CMPI1 R1, #3; JMPR cc_NE, back: leaves when R1 was 3, having counted it on to 4
    cpu = run(mov(1, 0) + [0x80, 0x31, 0x3D, 0xFE])
    assert cpu.r(1) == 4
    cpu = run(mov(1, CODE + 8) + [0x9C, 0x01, 0xE0, 0x52, 0xE0, 0x63])   # JMPI cc_UC, [R1] over MOV R2, #5
    assert cpu.r(2) == 0 and cpu.r(3) == 6


def test_bits():
    cpu = run([0x3F, 0x00,                 # BSET 0xFD00.3
               0x8A, 0x00, 0x01, 0x30,     # JB 0xFD00.3, over the next
               0xE0, 0x11,                 # MOV R1, #1
               0x0A, 0x00, 0xF0, 0x50,     # BFLDL 0xFD00, #0xF0, #0x50
               0x1A, 0x00, 0xA0, 0xF0,     # BFLDH 0xFD00, #0xF0, #0xA0
               0x4A, 0x00, 0xF2, 0x3F,     # BMOV R2.15, 0xFD00.3
               0x0F, 0xF3,                 # BSET R3.0
               0xAA, 0x00, 0x01, 0x30,     # JBC 0xFD00.3: clears it and jumps
               0xE0, 0x14])                # MOV R4, #1
    assert cpu.r(1) == 0 and cpu.r(4) == 0
    assert cpu.rw(0xFD00) == 0xA050 and cpu.r(2) == 0x8000 and cpu.r(3) == 1


def test_shifts_keep_the_rounding_bits():
    cpu = run(mov(1, 7) + [0x7C, 0x21])                                  # SHR R1, #2
    assert cpu.r(1) == 1 and flags(cpu) == C | V                         # the last bit out, and a 1 before it
    cpu = run(mov(1, 0x8001) + [0x5C, 0x11])                             # SHL R1, #1
    assert cpu.r(1) == 2 and flags(cpu) == C
    cpu = run(mov(1, 0x8004) + [0xBC, 0x21])                             # ASHR R1, #2
    assert cpu.r(1) == 0xE001 and flags(cpu) == N
    cpu = run(mov(1, 0x8001) + [0x1C, 0x41])                             # ROL R1, #4
    assert cpu.r(1) == 0x0018 and flags(cpu) == 0


def test_multiply_and_divide():
    cpu = run(mov(1, 0xFFFE) + mov(2, 3) + [0x0B, 0x12])                 # MUL R1, R2
    assert (cpu.rw(MDH), cpu.rw(MDL)) == (0xFFFF, 0xFFFA) and flags(cpu) == N
    cpu = run(mov(1, 0xFFFF) + [0x1B, 0x11])                             # MULU R1, R1
    assert (cpu.rw(MDH), cpu.rw(MDL)) == (0xFFFE, 0x0001) and flags(cpu) == V | N
    cpu = run([0xE6, 0x07, 0xF9, 0xFF] + mov(1, 2) + [0x4B, 0x11])       # MDL = -7; DIV R1
    assert (cpu.rw(MDL), cpu.rw(MDH)) == (0xFFFD, 0xFFFF)                # -3, remainder -1
    cpu = run([0xE6, 0x06, 0x01, 0x00, 0xE6, 0x07, 0x00, 0x00] + mov(1, 2) + [0x7B, 0x11])   # MD = 0x10000; DIVLU R1
    assert (cpu.rw(MDL), cpu.rw(MDH)) == (0x8000, 0)
    cpu = run([0xE6, 0x07, 0x05, 0x00] + mov(1, 0) + [0x5B, 0x11])       # DIVU by zero
    assert flags(cpu) == V and cpu.rw(MDL) == 5


def test_prior():
    cpu = run(mov(2, 0x0010) + [0x2B, 0x12])                             # PRIOR R1, R2
    assert cpu.r(1) == 11 and flags(cpu) == 0
    cpu = run(mov(1, 9) + mov(2, 0) + [0x2B, 0x12])
    assert cpu.r(1) == 0 and flags(cpu) == Z


def test_indirect_addressing():
    cpu = run(mov(1, 0xC200) + mov(2, 0xBEEF) + [0x88, 0x21,             # MOV [-R1], R2
                                                  0x98, 0x31,             # MOV R3, [R1+]
                                                  0xC4, 0x21, 0x10, 0x00,  # MOV [R1 + #0x10], R2
                                                  0xD4, 0x41, 0x10, 0x00])  # MOV R4, [R1 + #0x10]
    assert cpu.rw(0xC1FE) == 0xBEEF and cpu.r(3) == 0xBEEF and cpu.r(1) == 0xC200
    assert cpu.rw(0xC210) == 0xBEEF and cpu.r(4) == 0xBEEF


def test_switch_context():
    cpu = run(mov(1, 0x1111) + [0xC6, 0xF1, 0x34, 0x12, 0xFC, 0xF2])     # SCXT R1, #0x1234; POP R2
    assert cpu.r(1) == 0x1234 and cpu.r(2) == 0x1111


def test_trap_and_return():
    cpu = Cpu()
    cpu.flash[0x40:0x44] = bytes([0xFA, 0x00, 0x40, 0xC0])               # vector 0x10: JMPS 0, 0xC040
    cpu.write(0xC040, bytes([0xE0, 0x35, 0xFB, 0x88]))                   # MOV R5, #3; RETI
    sp = cpu.sp
    run(mov(1, 0x8000) + [0x9B, 0x20, 0xE0, 0x16], cpu)                  # (E and N set) TRAP #0x10; MOV R6, #1
    assert cpu.r(5) == 3 and cpu.r(6) == 1 and cpu.sp == sp


def test_an_undefined_opcode_stops_the_emulation():
    cpu = Cpu()
    cpu.write(CODE, bytes([0x8B, 0x00]))
    cpu.csp, cpu.ip = 0, CODE
    with pytest.raises(CpuError):
        cpu.run(10)
    assert cpu.pc == CODE


def test_hooks_see_register_accesses():
    seen = []
    cpu = Cpu()
    cpu.hook_write(0xFE20, lambda addr, value: seen.append(value))
    cpu.hook_read(0xFE22, lambda addr: 0x4242)
    run([0xE6, 0x10, 0x34, 0x12, 0xF2, 0xF1, 0x22, 0xFE], cpu)           # MOV reg 0x10, #0x1234; MOV R1, 0xFE22
    assert seen == [0x1234] and cpu.r(1) == 0x4242


# ---- interrupts ----------------------------------------------------------------
T4IC = 0xFF64                    # its request enters trap 0x24
ENABLE = [0xBF, 0x88]            # BSET IEN (PSW.11)


def interrupt_cpu():
    cpu = Cpu()
    cpu.flash[0x90:0x94] = bytes([0xFA, 0x00, 0x40, 0xC0])               # vector 0x24: JMPS 0, 0xC040
    cpu.write(0xC040, bytes([0x08, 0x51, 0xFB, 0x88]))                   # ADD R5, #1; RETI
    return cpu


def test_a_request_is_taken_when_its_level_beats_the_psw():
    cpu = interrupt_cpu()
    cpu.ww(T4IC, 0x44)                                                   # enabled, level 1
    cpu.request(T4IC)                                                    # the PSW does not let it in yet: it waits
    run([0xCC, 0x00] + ENABLE + [0xCC, 0x00, 0xE0, 0x16], cpu)
    assert cpu.r(5) == 1 and cpu.r(6) == 1
    assert cpu.interrupts_taken == {0x24: 1}
    assert cpu.rw(T4IC) == 0x44 and cpu.psw >> 12 == 0                   # request cleared, level back to 0


def test_level_zero_and_a_cleared_enable_keep_a_request_out():
    cpu = interrupt_cpu()
    run([0xE6, 0xB2, 0xC0, 0x00] + ENABLE + [0xCC, 0x00], cpu)           # requested and enabled, but level 0
    assert cpu.r(5) == 0
    cpu = interrupt_cpu()
    run([0xE6, 0xB2, 0xC4, 0x00, 0xCC, 0x00], cpu)                       # level 1, but IEN is off
    assert cpu.r(5) == 0 and cpu.rw(T4IC) == 0xC4


def test_an_atomic_sequence_holds_an_interrupt_off():
    cpu = interrupt_cpu()
    # ATOMIC #2 covers "MOV T4IC" (which raises the request) and "MOV R6, R5": R6 still sees 0
    run(ENABLE + [0xD1, 0x10, 0xE6, 0xB2, 0xC4, 0x00, 0xF0, 0x65, 0xCC, 0x00], cpu)
    assert cpu.r(6) == 0 and cpu.r(5) == 1


def test_pec_transfers_until_its_count_runs_out():
    cpu = interrupt_cpu()
    cpu.ww(0xC300, 0xBEEF)
    cpu.ww(0xFEC0, 0x0202)                                               # PECC0: 2 word transfers, destination counts up
    cpu.ww(0xFCE0, 0xC300)
    cpu.ww(0xFCE2, 0xC400)
    run([0xE6, 0xB2, 0x78, 0x00] + ENABLE + [0xCC, 0x00], cpu)           # T4IC: enabled, level 14, group 0 = PEC channel 0
    cpu.request(T4IC)
    run([0xCC, 0x00], cpu)
    assert (cpu.rw(0xC400), cpu.r(5), cpu.pec_transfers) == (0xBEEF, 0, 1)
    cpu.request(T4IC)
    run([0xCC, 0x00, 0xCC, 0x00], cpu)
    assert (cpu.rw(0xC402), cpu.rw(0xFCE2), cpu.rw(0xFEC0) & 0xFF) == (0xBEEF, 0xC404, 0)
    assert cpu.r(5) == 1 and cpu.pec_transfers == 2                      # the last transfer ends in the interrupt routine


def test_idle_waits_for_an_interrupt_and_time_jumps_to_it():
    cpu = interrupt_cpu()

    def event():
        cpu.next_event = 1 << 62
        cpu.request(T4IC)

    cpu.on_event = event
    cpu.next_event = 5000
    run([0xE6, 0xB2, 0x44, 0x00] + ENABLE + [0x87, 0x78, 0x87, 0x87, 0xE0, 0x16], cpu, steps=10_000)   # IDLE; MOV R6, #1
    assert cpu.r(5) == 1 and cpu.r(6) == 1
    assert 5000 <= cpu.instructions < 5010
