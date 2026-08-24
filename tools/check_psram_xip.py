#!/usr/bin/env python3
"""Structural verification for the PSRAM-execution app build (orcastra_psram).

This build cannot be smoke-tested without a launchable unit, so the invariants
that make it safe are asserted here instead. Run it after any change to
apps/orcastra/psram_xip_link/ or to the target's CMake block.

    python tools/check_psram_xip.py

Exit 0 = all invariants hold. Exit 1 = a real problem. Exit 2 = could not check.

WHAT IT CHECKS, AND WHY EACH ONE MATTERS

1. .text/.rodata have VMA == LMA inside PSRAM.
   That IS the scheme: execute in place. If VMA drifts into SRAM someone has
   re-added pico_set_binary_type(copy_to_ram) and the SRAM saving is gone.

2. Every timing-critical symbol is in SRAM.
   board_init() raises clk_sys to 250 MHz, which invalidates the QMI M1 timing
   for PSRAM until psram_reinitialize() completes. Anything executing in that
   gap is fetching through a mis-timed window. This is the check that stops the
   build from hard-faulting at boot; see psram_xip_link/pico_flash_region.ld.

3. Every audio-path object is in SRAM.
   Latency, not correctness -- 5.2 ms capture deadline (AGENTS.md invariant 15).

4. UI code stays in PSRAM.
   If main.c migrates to SRAM the scheme has stopped paying for itself.

5. The image fits the loader's 1 MB reservation at 0x11000000, and PSRAM data
   starts above it. Overlap here corrupts the code being executed, not a stale
   copy of it.
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ELF = ROOT / "build" / "apps" / "orcastra" / "orcastra_psram.elf"

PSRAM_IMG = (0x11000000, 0x11100000)   # 1 MB image reservation
PSRAM_DAT = 0x11100000
SRAM = (0x20000000, 0x20080000)

# Must be in SRAM or the boot-time re-clock fetches through a mis-timed window.
#
# These are the names the PSRAM-RESIDENT path actually links, which are NOT the
# ones you would guess from the stock boot flow. This list used to read
# board_init / set_sys_clock_khz / psram_reinitialize / psram_configure_params /
# flash_start_xip / xip_cache_clean_all -- and every one of them was ABSENT from
# the ELF, so the checker skipped all six and printed "All invariants hold" while
# verifying nothing. Why each went away:
#   board_init            -> we call board_init_inherited() via
#                            sd_entry_board_init_psram_resident() instead
#   set_sys_clock_khz     -> static inline in the SDK; folded into its caller
#   psram_* / flash_start_xip / xip_cache_clean_all
#                         -> PICO_RUNTIME_SKIP_INIT_PSRAM plus the no-op
#                            __wrap_psram_reinitialize() means the SDK's PSRAM
#                            init is never linked; the loader stub did it and
#                            sd_entry re-times the QMI by hand
# WALK_ANCHORS below is checked for existence, so this cannot rot silently again.
TIMING_CRITICAL = [
    "sd_entry_board_init_psram_resident", "board_init_inherited",
    "board_init_peripherals", "clock_configure", "clock_configure_internal",
    "set_sys_clock_pll", "vreg_set_voltage",
    "sd_entry_psram_devinfo", "sd_entry_irq_handover",
    "runtime_init_early_resets",
]
# Roots of the re-timing reachability walk. At least one MUST exist in the ELF,
# and the walk must cover a plausible number of functions -- see the assertions
# after the walk. An empty walk is a FAILURE, not a pass.
WALK_ANCHORS = ["sd_entry_board_init_psram_resident", "board_init_inherited",
                "board_init", "board_init_clk"]
# Must be in SRAM for the 5.2 ms deadline.
AUDIO_PATH = ["fx_process", "vox_process", "synth_process", "drums_process",
              "keys_process"]
# Must stay in PSRAM or the scheme is not saving anything.
MUST_STAY_PSRAM = ["main"]


def tool(name):
    for base in Path.home().glob(".pico-sdk/toolchain/*/bin"):
        p = base / f"arm-none-eabi-{name}.exe"
        if p.exists():
            return str(p)
        p = base / f"arm-none-eabi-{name}"
        if p.exists():
            return str(p)
    return None


def where(addr):
    if PSRAM_IMG[0] <= addr < PSRAM_IMG[1]:
        return "PSRAM-image"
    if addr >= PSRAM_DAT and addr < 0x12000000:
        return "PSRAM-data"
    if SRAM[0] <= addr < SRAM[1]:
        return "SRAM"
    if 0x20080000 <= addr < 0x20082000:
        return "SCRATCH"
    return f"other(0x{addr:08X})"


def main():
    if not ELF.exists():
        print(f"SKIP: {ELF} not built. Run: ninja -C build orcastra_psram")
        return 2
    readelf, nm = tool("readelf"), tool("nm")
    if not (readelf and nm):
        print("SKIP: arm-none-eabi toolchain not found")
        return 2

    fails, notes = [], []

    # ---- segments -------------------------------------------------------
    out = subprocess.run([readelf, "-l", str(ELF)], capture_output=True,
                         text=True).stdout
    segs = []
    for line in out.splitlines():
        # NB the Flg column is exactly three chars and CONTAINS SPACES
        # ("R E", "RW ", "RWE"), so it cannot be matched with \S+.
        m = re.match(r"\s+LOAD\s+0x\w+\s+0x(\w+)\s+0x(\w+)\s+0x(\w+)\s+"
                     r"0x(\w+)\s+([RWE ]{3})\s+0x\w+", line)
        if m:
            vma, lma, fsz, msz, flg = (int(m.group(1), 16), int(m.group(2), 16),
                                       int(m.group(3), 16), int(m.group(4), 16),
                                       m.group(5))
            segs.append((vma, lma, fsz, msz, flg))

    xip = [s for s in segs if s[4].startswith("R") and "E" in s[4]
           and PSRAM_IMG[0] <= s[0] < PSRAM_IMG[1] and s[2] > 4096]
    if not xip:
        fails.append("no large executable segment with VMA in PSRAM -- "
                     "is pico_set_binary_type(copy_to_ram) back?")
    for vma, lma, fsz, msz, flg in xip:
        if vma != lma:
            fails.append(f"PSRAM exec segment VMA 0x{vma:08X} != LMA 0x{lma:08X} "
                         "(not execute-in-place)")
        else:
            notes.append(f"exec-in-place segment 0x{vma:08X} size {fsz:,} (VMA==LMA)")

    top = max((s[0] + s[3]) for s in segs
              if PSRAM_IMG[0] <= s[0] < PSRAM_IMG[1]) if xip else 0
    if top > PSRAM_IMG[1]:
        fails.append(f"image top 0x{top:08X} exceeds the loader's 1 MB "
                     f"reservation ending 0x{PSRAM_IMG[1]:08X}")
    elif top:
        notes.append(f"image top 0x{top:08X}, "
                     f"{(PSRAM_IMG[1]-top)//1024} KB spare in the reservation")

    for vma, lma, fsz, msz, flg in segs:
        if vma >= PSRAM_DAT and msz and vma < top:
            fails.append(f"PSRAM data segment 0x{vma:08X} overlaps the image "
                         f"(top 0x{top:08X})")

    # ---- symbols --------------------------------------------------------
    out = subprocess.run([nm, str(ELF)], capture_output=True, text=True).stdout
    sym = {}
    for line in out.splitlines():
        m = re.match(r"^([0-9a-fA-F]{8})\s+[tTwW]\s+(\S+)$", line)
        if m:
            sym.setdefault(m.group(2), int(m.group(1), 16))

    def expect(names, want, why):
        for n in names:
            if n not in sym:
                notes.append(f"  (symbol {n} absent -- skipped)")
                continue
            got = where(sym[n])
            if got != want:
                fails.append(f"{n} is in {got}, must be {want} -- {why}")

    # A name-based list can only check names that EXIST. Six of the original
    # eight had quietly stopped existing (see TIMING_CRITICAL), so require that
    # a majority of the list resolves -- otherwise the "ok" lines below are
    # reporting on an empty set.
    present = [n for n in TIMING_CRITICAL if n in sym]
    if len(present) < len(TIMING_CRITICAL) // 2:
        fails.append(f"only {len(present)}/{len(TIMING_CRITICAL)} timing-critical "
                     "symbols exist in the ELF -- the list has rotted and this "
                     "check is not checking anything. Update TIMING_CRITICAL "
                     "against `nm` output.")

    expect(TIMING_CRITICAL, "SRAM",
           "executes while PSRAM QMI timing is stale during the boot re-clock")
    expect(AUDIO_PATH, "SRAM", "5.2 ms capture deadline")
    expect(MUST_STAY_PSRAM, "PSRAM-image",
           "UI code in SRAM defeats the purpose of this build")

    # ---- transitive re-timing reachability -------------------------------
    # The check above is a hand-written list, and a hand-written list missed
    # SEGGER_RTT_printf: board_init_clk() calls DIAG() between the clock change
    # and psram_reinitialize(), so the RTT implementation executes INSIDE the
    # mis-timed window. It hid because the direct callee is a long-branch VENEER
    # in SRAM that trampolines into PSRAM -- auditing direct callees showed
    # all-SRAM and looked correct. On hardware: progress bar fills, then a black
    # screen with no boot-trace markers and no recovery.
    #
    # So walk the call graph from board_init transitively, FOLLOWING VENEERS,
    # and assert nothing reachable lives in PSRAM. Mechanical, not remembered.
    dis = subprocess.run([tool("objdump"), "-d", "--no-show-raw-insn", str(ELF)],
                         capture_output=True, text=True).stdout
    bodies, cur = {}, None
    for line in dis.splitlines():
        m = re.match(r"^([0-9a-f]{8}) <([^>]+)>:", line)
        if m:
            cur = m.group(2); bodies[cur] = []
        elif cur:
            bodies[cur].append(line)

    def veneer_target(name):
        """Real target behind a long-branch veneer, or None if not a veneer.

        A veneer is literally `ldr.w pc,[pc]` followed by a `.word` holding the
        destination. Requiring the `ldr pc` matters: an earlier version of this
        function returned the first `.word` in ANY body, so every ordinary
        function with a literal pool looked like a veneer pointing at whatever
        its first constant happened to be -- which reported `panic -> 0x11016880`
        as a PSRAM call when panic was already correctly in SRAM.
        """
        body = bodies.get(name, [])
        if not any(re.search(r"\bldr(\.w)?\s+pc\s*,", ln) for ln in body):
            return None
        for ln in body:
            w = re.search(r"\.word\s+0x([0-9a-f]{8})", ln)
            if w:
                return int(w.group(1), 16) & ~1
        return None

    # `panic` is required to be in SRAM (it is in TIMING_CRITICAL's spirit and in
    # the exclude list), but we deliberately do NOT descend into it. Its reporting
    # path pulls in newlib stdio -- stdio_puts, weak_raw_vprintf, _exit, strlen --
    # and forcing all of that into SRAM would spend a lot of the memory this build
    # exists to reclaim on a path that only executes once we have ALREADY failed.
    # A panic inside the re-timing window is fatal either way; whether its message
    # makes it out is not worth the SRAM. Reported as a note so the tradeoff is
    # visible rather than silently ignored.
    DO_NOT_DESCEND = {"panic"}

    addr_to_name = {a: n for n, a in sym.items()}
    anchors = [a for a in WALK_ANCHORS if a in bodies]
    if not anchors:
        fails.append("none of the re-timing walk anchors "
                     f"{WALK_ANCHORS} exist in the ELF -- the reachability "
                     "check cannot run. This is the check that caught the "
                     "SEGGER_RTT_printf veneer; do not leave it dead.")
    seen, bad, frontier = set(), [], list(anchors)
    while frontier:
        fn = frontier.pop()
        if fn in seen or fn not in bodies or fn in DO_NOT_DESCEND:
            continue
        seen.add(fn)
        for ln in bodies[fn]:
            c = re.search(r"\bbl\s+[0-9a-f]+ <([^>+]+)>", ln)
            if not c:
                continue
            callee = c.group(1)
            tgt = veneer_target(callee)
            if tgt is not None:                      # trampoline: judge the real target
                real = addr_to_name.get(tgt, f"0x{tgt:08X}")
                if where(tgt) == "PSRAM-image":
                    bad.append(f"{fn} -> {callee} (veneer) -> {real} in PSRAM")
                frontier.append(real)
            else:
                if callee in sym and where(sym[callee]) == "PSRAM-image":
                    bad.append(f"{fn} -> {callee} in PSRAM")
                frontier.append(callee)

    if bad:
        for b in sorted(set(bad)):
            fails.append("re-timing path reaches PSRAM: " + b)
    elif anchors and len(seen) < 8:
        # "0 functions walked, none in PSRAM" reads as a pass and is not one.
        # The real sequence pulls in the clock, vreg and QMI helpers, so a walk
        # this shallow means the anchors resolved to stubs or the disassembly
        # parse broke -- either way there is no coverage to report.
        fails.append(f"re-timing walk from {anchors} covered only {len(seen)} "
                     "function(s) -- too shallow to be real coverage")
    else:
        notes.append(f"re-timing call graph from {', '.join(anchors)}: "
                     f"{len(seen)} functions walked (veneers followed), "
                     "none in PSRAM")
        notes.append("panic is in SRAM but its stdio reporting path is not "
                     "(deliberate - see DO_NOT_DESCEND)")

    # ---- SRAM headroom --------------------------------------------------
    sram_top = max((s[0] + s[3]) for s in segs
                   if SRAM[0] <= s[0] < SRAM[1])
    free = SRAM[1] - sram_top
    notes.append(f"SRAM top 0x{sram_top:08X}, free {free:,} bytes "
                 f"({free/1024:.1f} KB)")
    if free < 8 * 1024:
        fails.append(f"only {free:,} bytes of SRAM free -- the RAM-resident set "
                     "in default_text_excludes.incl has grown too large")

    # ---- report ---------------------------------------------------------
    print("orcastra_psram structural check\n" + "-" * 34)
    for n in notes:
        print(f"  ok   {n}")
    if fails:
        print()
        for f in fails:
            print(f"  FAIL {f}")
        print(f"\n{len(fails)} problem(s).")
        return 1
    print("\nAll invariants hold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
