# c166-emu

The C166-family CPU core that `gs20-emu` (Siemens GS20 gearbox, C167CR)
and `ms43-emu` (Siemens MS43 DME, C167CS) both run on. Unicorn has no
C166, so this is an interpreter of the whole instruction set, written
from Infineon's "Instruction Set Manual for the C166 Family" (V2.0), with
the chip's interrupt system and PEC. It started as `gs20emu/cpu.py`.

- `c166/cpu.py`: registers, data paging (DPP, EXTP/EXTS/EXTR), the
  instructions, traps and interrupts, PEC, IDLE, events on the
  instruction count.
- `c166/sfr.py`: the SFR and ESFR names by address.
- `c166/periph.py`: the chip's peripherals as the two boards use them: the
  timers T2-T6 and the CAPCOM timers T0/T1 (T7/T8), the serial port (a K
  line), the SSC, the CAN module, the A/D converter, and an Am29F400BB's
  command interface. A device takes the board it is on, which gives it
  `cpu`, `ips`, `fcpu`, `plan()`, `unsupported` and `read16`.

What belongs to a board stays in its project: which devices it has and
what is on the other end of them, its clock, and the memory map above
segment 0, which the board sets in 4 KB pages:

    from c166.cpu import Cpu
    cpu = Cpu(flash_size=0x80000, default_map=False)
    cpu.map_flash(0x010000, 0x100000)      # a 512 KB chip decoded below 1 MB
    cpu.map_ram(0x040000, 0x2000)          # 8 KB of external RAM over it

Without `default_map=False` the map is the GS20's: flash wherever address
bit 19 is set.

## Setup

The emulators install this into their own environments:

    ../gs20-emu/.venv/bin/pip install -e ../c166-emu
    ../gs20-emu/.pypy/bin/pypy3 -m pip install -e ../c166-emu

Its own tests:

    .venv/bin/pytest            # or ../gs20-emu/.pypy/bin/pypy3 -m pytest
