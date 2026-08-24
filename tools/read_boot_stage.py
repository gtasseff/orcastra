#!/usr/bin/env python3
"""Read a launched app's boot-stage breadcrumb over SWD, without halting.

    python tools/read_boot_stage.py [orcastra_psram|orcastra_sd]

WHY THIS EXISTS. An SD-launched app that dies during early init leaves nothing
to read: the screen is black, the SEGGER RTT buffer is still all zeros because no
DIAG() completed, and once the CPU escalates to LOCKUP the PC reads 0xEFFFFFFE
and tells you nothing about where it went wrong.

Hardware breakpoints are the obvious instrument and they fought us: this target
has TWO Cortex-M cores, so a scripted halt/resume pair resumes only one and
errors out -- which leaves the display FROZEN, and then nobody can tap the menu
to launch the app you are trying to catch. Two attempts produced no data.

So sd_entry.c writes a monotonically increasing stage number to a fixed
never-zeroed SRAM word (`sd_boot_stage`, in .uninitialized_data so crt0 cannot
erase the evidence). This reads it with a single `mdw` while the target runs --
no halt, so the GUI keeps working and a hung app can still be three-finger reset.

The address is read from the ELF each time rather than hardcoded, so it cannot
rot when the layout shifts.
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

STAGES = {
    0:  "nothing ran (or a fresh boot) -- crt0 never reached our runtime hooks",
    1:  "sd_entry_irq_handover entered: crt0 copied us out and runtime init began",
    2:  "survived the IRQ handover loop over NUM_IRQS",
    3:  "sd_entry_psram_devinfo entered",
    4:  "flash_devinfo CS1 was unset, about to restore it",
    5:  "OTP-backed devinfo writes survived",
    6:  "FIRST DIAG COMPLETED -- RTT is alive from here on",
    7:  "CS1 was already set by the loader; nothing to restore",
    8:  "runtime_init_early_resets (OUR override) entered",
    9:  "SURVIVED the early resets with IO_BANK0/PADS_BANK0 spared -- the bug that killed attempts 1-3",
    10: "reached main(); ALL runtime init succeeded",
    20: "board init entered (PSRAM-resident path)",
    21: "vreg raised to 1.25 V, settled",
    22: "QMI M1 timing pre-loaded with a value legal at BOTH clocks",
    23: "SURVIVED set_sys_clock_khz(250000) -- this is the step that used to lock up",
    24: "QMI timing tightened for 250 MHz",
    25: "clk_peri re-sourced from clk_sys",
    26: "board_init_inherited() returned: SPI, GPIO, backlight, I2C1, ioexp up",
    11: "board init returned -- past the whole clock/PSRAM sequence",
}


def tool(name):
    for base in Path.home().glob(".pico-sdk/openocd/*"):
        exe = base / f"{name}.exe"
        if exe.exists():
            return str(exe), str(base / "scripts")
    return None, None


def sym_addr(elf, name):
    for base in Path.home().glob(".pico-sdk/toolchain/*/bin"):
        nm = base / "arm-none-eabi-nm.exe"
        if not nm.exists():
            continue
        out = subprocess.run([str(nm), str(elf)], capture_output=True, text=True).stdout
        for line in out.splitlines():
            m = re.match(r"^([0-9a-fA-F]{8})\s+\S+\s+(\S+)$", line)
            if m and m.group(2) == name:
                return int(m.group(1), 16)
    return None


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "orcastra_psram"
    elf = ROOT / "build" / "apps" / "orcastra" / f"{target}.elf"
    if not elf.exists():
        print(f"no ELF at {elf} -- build {target} first")
        return 2
    addr = sym_addr(elf, "sd_boot_stage")
    if addr is None:
        print("sd_boot_stage not found in the ELF "
              "(is ORCASTRA_SD_BOOT_TRACE defined for this target?)")
        return 2
    oocd, scripts = tool("openocd")
    if not oocd:
        print("openocd not found under ~/.pico-sdk")
        return 2

    cfg = ROOT / "tools" / "openocd" / "fw2_display_if0.cfg"
    # NO halt: reading memory on a running target is non-perturbing, keeps the
    # GUI alive, and works even when the core has escalated to LOCKUP.
    out = subprocess.run(
        [oocd, "-s", scripts, "-f", str(cfg),
         "-c", "init", "-c", f"mdw 0x{addr:08X}", "-c", "shutdown"],
        capture_output=True, text=True)
    m = re.search(r"0x[0-9a-f]{8}:\s+([0-9a-f]{8})", out.stdout + out.stderr)
    if not m:
        print("could not read the target -- is the unit connected?")
        print((out.stdout + out.stderr)[-400:])
        return 1

    val = int(m.group(1), 16)
    print(f"LAST stage reached: {val}"
          f"{'' if val < 0x100 else '  (not a stage code -- stale or uninitialised)'}")
    print(f"  {STAGES.get(val, 'UNKNOWN stage value')}")

    # The ORDER-INDEPENDENT view. sd_boot_stage alone is ambiguous, because the
    # two EARLIEST hooks run in link order, not numbering order -- a final value
    # of 2 once looked like "died before 3" when devinfo may have already run.
    # The bitmask cannot be fooled that way: every visited stage leaves a bit.
    vaddr = sym_addr(elf, "sd_boot_visited")
    if vaddr is not None:
        out2 = subprocess.run(
            [oocd, "-s", scripts, "-f", str(cfg),
             "-c", "init", "-c", f"mdw 0x{vaddr:08X}", "-c", "shutdown"],
            capture_output=True, text=True)
        m2 = re.search(r"0x[0-9a-f]{8}:\s+([0-9a-f]{8})", out2.stdout + out2.stderr)
        if m2:
            mask = int(m2.group(1), 16)
            hit = [b for b in range(32) if mask & (1 << b)]
            print(f"\nALL stages visited (mask 0x{mask:08X}): "
                  f"{hit if hit else 'none'}")
            for b in hit:
                print(f"    {b:>2}  {STAGES.get(b, '?')}")
            missing = [k for k in sorted(STAGES) if k not in hit and k <= max(hit or [0])]
            if missing:
                print(f"  NOT visited below the high-water mark: {missing}")
            nxt = [k for k in sorted(STAGES) if k > max(hit or [0])]
            if nxt:
                print(f"\n  FIRST STAGE NEVER REACHED: {nxt[0]} -- {STAGES[nxt[0]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
