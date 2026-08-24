#!/usr/bin/env python3
"""fw — fw2-audio-demo task runner (cross-platform), adapted from wilibsp/tools/fw.py.

Commands:
  fw build [app]     configure+build an app for the RP2350B target (default orcastra)
  fw flash [app]     program the app over the cmsis-dap debug probe via OpenOCD
  fw rtt [-s N]      stream SEGGER RTT diagnostics (N seconds, 0 = until Ctrl+C)
  fw new-app <name>  scaffold apps/<name> from apps/template

Unlike wilibsp's fw, `build` self-configures: it locates the Pico VS Code
extension toolchain under ~/.pico-sdk (SDK 2.3.0, ARM GCC, CMake, Ninja,
picotool) and injects it into the subprocess environment, so no global env
setup is required. Add --print to print commands instead of running them.
"""
import argparse, os, pathlib, shutil, socket, subprocess, sys, time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_APP = "orcastra_psram"
OPENOCD_CFG = str(REPO_ROOT / "tools" / "openocd" / "freewili2.cfg")
RTT_PORT = 9090
# RP2350 SRAM is 0x20000000..0x20082000; scan the whole range for the RTT block.
RTT_SETUP = 'rtt setup 0x20000000 0x82000 "SEGGER RTT"'
PICO_ROOT = pathlib.Path.home() / ".pico-sdk"
SDK_VERSION = "2.3.0"


def _newest(pattern_root, glob):
    """Newest matching path under pattern_root, or None."""
    if not pattern_root.is_dir():
        return None
    return next(iter(sorted(pattern_root.glob(glob), reverse=True)), None)


def build_env():
    """Environment with the ~/.pico-sdk toolchain injected (this process only)."""
    env = os.environ.copy()
    sdk = PICO_ROOT / "sdk" / SDK_VERSION
    toolchain = _newest(PICO_ROOT / "toolchain", "*")
    cmake_bin = _newest(PICO_ROOT / "cmake", "v*/bin")
    ninja_dir = _newest(PICO_ROOT / "ninja", "v*")
    picotool = _newest(PICO_ROOT / "picotool", "*/picotool")
    missing = [str(p) for p in (sdk, toolchain, cmake_bin, ninja_dir) if p is None or not p.exists()]
    if missing:
        sys.exit(f"missing Pico toolchain paths (install the Pico VS Code extension): {missing}")
    env["PICO_SDK_PATH"] = str(sdk)
    env["PICO_TOOLCHAIN_PATH"] = str(toolchain)
    prepend = [str(cmake_bin), str(ninja_dir), str(toolchain / "bin")]
    if picotool:
        prepend.append(str(picotool))
        env["picotool_DIR"] = str(picotool)
    env["PATH"] = os.pathsep.join(prepend + [env.get("PATH", "")])
    return env


def build_commands(app):
    cmds = []
    if not (REPO_ROOT / "build" / "CMakeCache.txt").exists():
        configure = ["cmake", "--preset", "target"]
        # Prebuilt host tools shipped by the Pico VS Code extension — without
        # these the SDK tries to compile them and needs a host C++ compiler.
        picotool = _newest(PICO_ROOT / "picotool", "*/picotool")
        pioasm = _newest(PICO_ROOT / "tools", "*/pioasm")
        if picotool:
            configure.append(f"-Dpicotool_DIR={picotool}")
        if pioasm:
            configure.append(f"-Dpioasm_DIR={pioasm}")
        cmds.append(configure)
    cmds.append(["cmake", "--build", "--preset", "target", "--target", app])
    return cmds


def _openocd():
    """(exe, scripts_dir) for the Pico-SDK OpenOCD; falls back to PATH."""
    root = PICO_ROOT / "openocd"
    if root.is_dir():
        exe_name = "openocd.exe" if sys.platform == "win32" else "openocd"
        for ver in sorted(root.iterdir(), reverse=True):
            exe, scripts = ver / exe_name, ver / "scripts"
            if exe.exists():
                return str(exe), (str(scripts) if scripts.is_dir() else None)
    return "openocd", None


def _openocd_base():
    exe, scripts = _openocd()
    cmd = [exe]
    if scripts:
        cmd += ["-s", scripts]
    return cmd + ["-f", OPENOCD_CFG]


# DMA quiesce — THE fix for the flash "wedge" (root-caused 2026-07-17).
# OpenOCD's reset on this chip is core-only: the app's free-running DMA
# channels keep going. The PDM capture ring (DMA ch1, 32 KiB, endless
# transfer count) overlaps OpenOCD's flash work area at 0x20010000, so the
# uploaded flash algorithm gets sprayed with PDM bits mid-run -> "Failed to
# invoke flash programming code" / "timed out while waiting for target
# halted" / garbage QSPI ids. Bench-proven: after `reset halt`, ch1's
# WRITE_ADDR kept advancing through the halted state.
# Fix: halt, ABORT + DISABLE all 16 DMA channels (CHAN_ABORT, then each
# CHx_CTRL_TRIG = 0). The cores stay halted through program's own reset, so
# nothing re-arms the DMA before flashing completes.
DMA_QUIESCE = ["init", "reset halt",
               "mww 0x50000464 0xffff"] + [           # CHAN_ABORT: all 16
    "mww 0x%08x 0" % (0x5000000c + ch * 0x40) for ch in range(16)  # EN=0
]


def flash_command(app):
    elf = f"build/apps/{app}/{app}.elf"
    cmd = _openocd_base()
    for c in DMA_QUIESCE:
        cmd += ["-c", c]
    return cmd + ["-c", f"program {elf} verify reset exit"]


def rtt_command():
    return _openocd_base() + [
        "-c", "init", "-c", RTT_SETUP, "-c", "rtt start",
        "-c", f"rtt server start {RTT_PORT} 0"]


def new_app(name, repo_root=REPO_ROOT):
    src = pathlib.Path(repo_root) / "apps" / "template"
    dest = pathlib.Path(repo_root) / "apps" / name
    if dest.exists():
        raise FileExistsError(dest)
    shutil.copytree(src, dest)
    cml = dest / "CMakeLists.txt"
    cml.write_text(cml.read_text().replace("template", name))
    return dest


def _run(cmds, do_print, env=None):
    if isinstance(cmds[0], str):
        cmds = [cmds]
    for c in cmds:
        if do_print:
            print(" ".join(c))
        else:
            subprocess.run(c, cwd=REPO_ROOT, check=True, env=env)


def run_rtt(seconds=0):
    """Start OpenOCD's RTT server (attached, no flash) and stream channel 0."""
    proc = subprocess.Popen(rtt_command(), cwd=REPO_ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sock = None
    try:
        time.sleep(2)  # let OpenOCD attach and locate the RTT control block
        if proc.poll() is not None:
            print("openocd exited early — is the debug probe connected?", file=sys.stderr)
            return 1
        try:
            sock = socket.create_connection(("127.0.0.1", RTT_PORT), timeout=5)
        except OSError as e:
            print(f"could not connect to RTT server on {RTT_PORT}: {e}", file=sys.stderr)
            return 1
        sock.settimeout(0.5)
        deadline = time.time() + seconds if seconds > 0 else None
        print(f"--- RTT connected (port {RTT_PORT}); Ctrl+C to stop ---", file=sys.stderr)
        while deadline is None or time.time() < deadline:
            try:
                data = sock.recv(4096)
                if not data:
                    break
                sys.stdout.write(data.decode("ascii", "replace"))
                sys.stdout.flush()
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        _openocd_shutdown(proc)
    return 0


def _openocd_shutdown(proc):
    """Shut OpenOCD down via its TCL RPC port; hard-kill only as a last resort.

    TerminateProcess'ing OpenOCD (the old behavior) abandons the on-board
    CMSIS-DAP probe mid-session; the NEXT `fw flash` then fails its ROM
    function batch ("timed out while waiting for target halted", garbage
    QSPI flash id) until the board gets a FULL power cycle (USB + battery
    jumper) — AGENTS.md invariant 14. A clean `shutdown` releases the
    probe properly."""
    if proc.poll() is not None:
        return
    try:
        with socket.create_connection(("127.0.0.1", 6666), timeout=2) as s:
            s.sendall(b"shutdown\x1a")           # TCL RPC, 0x1a-terminated
        proc.wait(timeout=5)
        return
    except (OSError, subprocess.TimeoutExpired):
        pass
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def main(argv=None):
    p = argparse.ArgumentParser(prog="fw")
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("build", "flash"):
        sp = sub.add_parser(name); sp.add_argument("app", nargs="?", default=DEFAULT_APP)
        sp.add_argument("--print", dest="show", action="store_true")
    sp = sub.add_parser("rtt")
    sp.add_argument("--print", dest="show", action="store_true")
    sp.add_argument("-s", "--seconds", type=int, default=0,
                    help="capture for N seconds then exit (0 = until Ctrl+C)")
    sp = sub.add_parser("new-app"); sp.add_argument("name")

    a = p.parse_args(argv)
    if a.cmd == "build":
        _run(build_commands(a.app), a.show, env=None if a.show else build_env())
    elif a.cmd == "flash":
        _run(flash_command(a.app), a.show)
    elif a.cmd == "rtt":
        if a.show:
            _run(rtt_command(), True)
        else:
            return run_rtt(a.seconds)
    elif a.cmd == "new-app":
        print("created", new_app(a.name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
