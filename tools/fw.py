#!/usr/bin/env python3
"""fw — Orcastra task runner (cross-platform), adapted from wilibsp/tools/fw.py.

Commands:
  fw build [target]  configure+build a target for the RP2350B (default orcastra)
  fw rtt [-s N]      stream SEGGER RTT diagnostics (N seconds, 0 = until Ctrl+C)

Targets are orcastra (ships; executes from PSRAM) and orcastra_sd, both
built from apps/orcastra -- so a target name is NOT an app directory name.
Artifacts land in build/apps/orcastra/.

There is no `flash` command, by design. Both targets load at 0x11000000 and are
installed by copying the .uf2 to /apps on the SD card, or with wilibsp's
`fw install-app`. Programming an app into QSPI flash over the probe would
overwrite the stock DISPLAY firmware, which is the bootloader that launches apps
from the card. The probe is used by `fw rtt` and by tools/agentio.py, neither of
which writes to the device.

Unlike wilibsp's fw, `build` self-configures: it locates the Pico VS Code
extension toolchain under ~/.pico-sdk (SDK 2.3.0, ARM GCC, CMake, Ninja,
picotool) and injects it into the subprocess environment, so no global env
setup is required. Add --print to print commands instead of running them.
"""
import argparse, os, pathlib, socket, subprocess, sys, time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_APP = "orcastra"
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


def rtt_command():
    return _openocd_base() + [
        "-c", "init", "-c", RTT_SETUP, "-c", "rtt start",
        "-c", f"rtt server start {RTT_PORT} 0"]


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

    TerminateProcess'ing OpenOCD abandons the on-board CMSIS-DAP probe
    mid-session; the next probe session (this command again, or
    tools/agentio.py) then fails to attach — "timed out while waiting for
    target halted", garbage QSPI ids — until the board gets a FULL power
    cycle (USB + battery jumper). A clean `shutdown` releases the probe
    properly."""
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
    sp = sub.add_parser("build")
    sp.add_argument("app", nargs="?", default=DEFAULT_APP)
    sp.add_argument("--print", dest="show", action="store_true")
    sp = sub.add_parser("rtt")
    sp.add_argument("--print", dest="show", action="store_true")
    sp.add_argument("-s", "--seconds", type=int, default=0,
                    help="capture for N seconds then exit (0 = until Ctrl+C)")

    a = p.parse_args(argv)
    if a.cmd == "build":
        _run(build_commands(a.app), a.show, env=None if a.show else build_env())
    elif a.cmd == "rtt":
        if a.show:
            _run(rtt_command(), True)
        else:
            return run_rtt(a.seconds)
    return 0


if __name__ == "__main__":
    sys.exit(main())
