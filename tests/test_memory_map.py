"""The pages above segment 0: open, flash (masked to the chip's size) and external RAM."""
from c166.cpu import Cpu, CpuError

import pytest


def test_default_map_is_flash_where_bit_19_is_set():
    cpu = Cpu()
    cpu.flash[0x1234:0x1236] = b"\x34\x12"
    assert cpu.rw(0x081234) == 0x1234
    assert cpu.rw(0x181234) == 0x1234            # the chip ignores the address lines above it
    assert cpu.rw(0x041234) == 0xFFFF and 0x041234 in cpu.open_reads


def test_a_board_can_remap():
    cpu = Cpu(default_map=False)
    cpu.flash[0x61234:0x61236] = b"\x34\x12"
    cpu.map_flash(0x010000, 0x100000)
    cpu.map_ram(0x040000, 0x2000)
    assert cpu.rw(0x061234) == 0x1234 and cpu.rw(0x0E1234) == 0x1234     # 512 KB chip: A19 ignored
    cpu.ww(0x040010, 0xBEEF)
    cpu.wb(0x040013, 0x5A)
    assert cpu.rw(0x040010) == 0xBEEF and cpu.rb(0x040013) == 0x5A
    assert cpu.ext_ram[0x040010:0x040012] == b"\xef\xbe"
    assert cpu.rw(0x042000) != 0xBEEF and cpu.rw(0x100000) == 0xFFFF     # beyond the RAM: flash again; beyond 1 MB: nothing


def test_code_runs_from_external_ram():
    cpu = Cpu(default_map=False)
    cpu.map_ram(0x040000, 0x1000)
    code = bytes([0xE6, 0xF1, 0x34, 0x12])       # MOV R1, #0x1234
    cpu.write(0x040100, code)
    cpu.csp, cpu.ip = 0x04, 0x0100
    cpu.run(1)
    assert cpu.r(1) == 0x1234 and cpu.pc == 0x040104


def test_executing_an_open_page_stops():
    cpu = Cpu(default_map=False)
    cpu.csp, cpu.ip = 0x02, 0
    with pytest.raises(CpuError, match="executing where nothing is"):
        cpu.run(1)
