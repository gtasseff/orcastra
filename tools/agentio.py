#!/usr/bin/env python3
"""agentio — run wilibsp's fw.py (screenshot/touch/press/type) with a pinned
RTT control-block address.

wilibsp's fw.py locates the SEGGER RTT control block by scanning all of SRAM
for the "SEGGER RTT" magic. Reflashing moves _SEGGER_RTT (the code/bss layout
shifts) but does NOT clear RAM, so a previous build's control block — magic
string intact, buffer pointers dangling — survives at its old address. The
scan finds whichever stale block sits lowest and the host then reads garbage
through dead pointers (symptom: screenshot returns TSV-looking noise or times
out until a full power cycle).

Fix: read _SEGGER_RTT straight out of the current ELF with nm and constrain
OpenOCD's search to exactly that address. Stale blocks are then invisible no
matter how many reflashes have happened since the last power cycle.

Usage (from repo root):  python tools/agentio.py screenshot -o shot.png
                         python tools/agentio.py touch 240 160
All arguments are forwarded verbatim to wilibsp's fw.py.
"""
import importlib.util, pathlib, re, subprocess, sys

REPO = pathlib.Path(__file__).resolve().parents[1]
WILIBSP_FW = REPO / "external" / "wilibsp" / "tools" / "fw.py"
# The PSRAM-resident target is the only one with SRAM room for the harness, so
# it is the only one worth pinning against. NOTE the asymmetry that bit us: the
# target is `orcastra_psram` (hence orcastra_psram.elf) but its UF2 is named
# plainly `orcastra.uf2`, because the on-device App Explorer lists FILENAMES and
# "orcastra" is what should appear in the menu. Pointing this at orcastra.elf --
# the name that matches the UF2 rather than the target -- made every harness
# command exit with "need toolchain nm and .../orcastra.elf" instead of running.
# The fallbacks let it still work against a copy_to_ram build if one is current.
ELF_CANDIDATES = ["orcastra_psram.elf", "orcastra_sd.elf", "orcastra_swd.elf"]
PICO_ROOT = pathlib.Path.home() / ".pico-sdk"


def elf_path():
    d = REPO / "build" / "apps" / "orcastra"
    for name in ELF_CANDIDATES:
        if (d / name).exists():
            return d / name
    return d / ELF_CANDIDATES[0]        # for the error message below


def rtt_block_addr():
    nm = next(iter(sorted((PICO_ROOT / "toolchain").glob("*/bin/arm-none-eabi-nm*"))), None)
    elf = elf_path()
    if nm is None or not elf.exists():
        sys.exit(f"need toolchain nm and {elf} (run: python tools/fw.py build)")
    out = subprocess.run([str(nm), str(elf)], capture_output=True, text=True, check=True).stdout
    m = re.search(r"^([0-9a-fA-F]+) [BbDd] _SEGGER_RTT$", out, re.M)
    if not m:
        sys.exit("_SEGGER_RTT not found in ELF symbol table")
    return int(m.group(1), 16)


def main(argv):
    spec = importlib.util.spec_from_file_location("wilibsp_fw", WILIBSP_FW)
    fw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fw)
    addr = rtt_block_addr()
    # Search window = the exact block only; stale magics elsewhere can't match.
    fw.RTT_SETUP = f'rtt setup 0x{addr:08x} 0x30 "SEGGER RTT"'
    return fw.main(argv)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
