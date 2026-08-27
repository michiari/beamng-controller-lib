import ctypes
import logging
import platform
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path, PureWindowsPath, PurePosixPath
from typing import Callable, IO, List, Optional

import numpy as np
from beamngpy import BeamNGpy

logger = logging.getLogger(__name__)


def to_local_posix(path: PureWindowsPath) -> Path:
    """
    Translates the given path to a path usable by the local filesystem.

    On WSL, drive-lettered Windows paths (e.g., C:\\Users\\...) must be translated to their
    WSL mount equivalent (/mnt/c/Users/...) before being handed to local tools, such as rsync.

    Args:
        path: The path to translate.

    Returns:
        The translated path.
    """

    if "wsl" in platform.uname().release.lower():
        result = subprocess.run(
            ["wslpath", "-u", path.as_posix()],
            capture_output=True, text=True, check=True,
        )
        return Path(result.stdout.strip())
    return Path(path.as_posix())


def open_beamng_with_retry(
    host: str,
    port: int,
    home: str,
    user: str,
    process: Optional[subprocess.Popen] = None,
    timeout: float = 600.0,
    probe_interval: float = 5.0,
) -> BeamNGpy:
    """
    Connects a BeamNGpy client to a running BeamNG instance, retrying the full handshake until it
    succeeds or `timeout` elapses. Exits early if the supplied BeamNG process dies, so callers
    can restart BeamNG instead of waiting the full timeout on a dead port.

    Args:
        host: Host where BeamNG is listening.
        port: Port where BeamNG is listening.
        home: BeamNG installation directory, forwarded to BeamNGpy.
        user: BeamNG per-worker user directory, forwarded to BeamNGpy.
        process: BeamNG process handle from the local launcher, used to detect crashes during
            startup. Pass None when BeamNG runs on a remote host (no local handle).
        timeout: Maximum total seconds to wait before giving up.
        probe_interval: Seconds to wait between handshake attempts.

    Returns:
        An open BeamNGpy instance ready for use.

    Raises:
        RuntimeError: If the handshake does not complete within `timeout` seconds, or if the BeamNG
            process exits before the handshake completes.
    """

    logger.debug(f"⏳ Connecting to BeamNG at {host}:{port} (handshake timeout {timeout:.0f}s)...")
    deadline = time.monotonic() + timeout
    attempts = 0
    last_error: Optional[BaseException] = None
    while True:
        if process is not None and process.poll() is not None:
            raise RuntimeError(
                f"❌ BeamNG process (pid {process.pid}) exited with code {process.returncode} "
                f"before handshake on {host}:{port}; see the per-worker beamng_stderr.log."
            )

        attempts += 1
        bng = BeamNGpy(host=host, port=port, home=home, user=user)
        try:
            bng.open(launch=False)
        except Exception as e:
            last_error = e
            try:
                bng.close()
            except Exception:
                pass
        else:
            elapsed = timeout - (deadline - time.monotonic())
            logger.debug(f"✅ BeamNG handshake completed after {elapsed:.1f}s ({attempts} attempt(s)).")
            return bng

        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"❌ Failed to connect to BeamNG at {host}:{port} after {attempts} attempts within "
                f"{timeout:.0f}s (last error: {last_error})"
            )
        time.sleep(probe_interval)


def start_beamng_linux(
    runnable_path: PurePosixPath, user_path: PurePosixPath, port: int
) -> tuple[str, subprocess.Popen]:
    """
    Starts BeamNG on Linux.
    Args:
        runnable_path: The path to the BeamNG executable.
        port: The port to use for communication with BeamNG.
        user_path: The path to the BeamNG user directory.

    Returns:
        The IP address the BeamNG process listens to and a handle to the started process.
    """

    logger.debug("🚀 Starting BeamNG on Linux.")
    beamng_path_as_posix = runnable_path.as_posix()

    cli_args = [
        "-nosteam",
        "-userpath",
        str(user_path.as_posix()),
        "-tcom",
        "-tport",
        str(port),
        "-gfx",
        "null",
        "-headless",
        "-batch",
    ]

    process = _start_beamng([beamng_path_as_posix] + cli_args, log_path=_beamng_log_path(user_path))
    logger.debug("✅ Started BeamNG successfully.")
    return "localhost", process


def start_beamng_wsl(
    runnable_path: PureWindowsPath, port: int, user_path: PureWindowsPath, headless=False
) -> tuple[str, subprocess.Popen]:
    """
    Starts BeamNG on Windows from within the WSL environment.
    Args:
        runnable_path: The path to the BeamNG executable on Windows.
        port: The port to use for communication with BeamNG.
        user_path: The path to the BeamNG executable on Windows.
        headless: Whether to run BeamNG in headless mode.

    Returns:
        The IP address the BeamNG process listens to (within WSL) and a handle to the started process.
    """

    logger.debug("🚀 Starting BeamNG from WSL.")
    # Convert the WindowsPath to a PosixPath, move it to the partition following the WSL standard
    beamng_path_as_posix = runnable_path.as_posix()
    drive_on_wsl = f"/mnt/{beamng_path_as_posix.split(':')[0].lower()}"
    beamng_path_on_wsl = f"{drive_on_wsl}" + "".join(beamng_path_as_posix.split(":")[1:])

    # Note the 0.0.0.0 is hardcoded and means that this connection will listen on all the ports on wsl
    cli_args = [
        "-nosteam",
        "-userpath",
        str(user_path.as_posix()),
        "-console",
        "-tcom-listen-ip",
        "0.0.0.0",
        "-batch",
        "-lua",
        f"extensions.load('tech/techCore');tech_techCore.openServer({port})",
    ]

    if headless:
        cli_args.append("-headless")
        cli_args.append("-gfx")
        cli_args.append("null")

    process = _start_beamng([beamng_path_on_wsl] + cli_args, log_path=_beamng_log_path(user_path))

    # Search for windows host ip so the BeamNG instance can be set up to listen on that port.
    wsl_ip_on_win = get_windows_host_ip()
    logger.debug(f"✅ Detected WSL IP on the Windows host: {wsl_ip_on_win}")

    logger.debug("✅ Started BeamNG successfully.")
    return wsl_ip_on_win, process


def get_windows_host_ip() -> str:
    """
    Detect the Windows host IP from inside WSL as this is the IP where BeamNG will listen to.

    In WSL2 mirrored networking mode, WSL shares the Windows host's interfaces and BeamNG is reachable on 127.0.0.1. In NAT mode, the host is
    reached via the WSL default gateway.

    Returns:
        The Windows host IP from WSL.
    """

    try:
        with open("/proc/net/dev") as proc_net_dev:
            if "loopback0:" in proc_net_dev.read():
                return "127.0.0.1"
    except OSError:
        pass

    try:
        result = subprocess.run(
            ["bash", "-c", "ip route | grep default | awk '{print $3}'"], capture_output=True, text=True, check=True
        )
        ip = result.stdout.strip().split()[0]
        return ip
    except Exception as e:
        raise RuntimeError(f"❌ Could not determine Windows host IP: {e}")


def _beamng_log_path(user_path: PureWindowsPath) -> Path:
    """Resolves the per-worker BeamNG stderr log path next to the user dir."""
    local_user = to_local_posix(user_path)
    return local_user.parent / f"{local_user.name}_beamng_stderr.log"


def _start_beamng(cli_command: list[str], log_path: Optional[Path] = None) -> subprocess.Popen:
    """
    Starts beamng with the given CLI command.

    Args:
        cli_command: The CLI command to start beamng with.
        log_path: Optional file path where BeamNG's stdout/stderr are appended. When None the
            output is discarded. Capturing the stream is essential for diagnosing silent crashes
            that happen after the process-poll check below.

    Returns:
        A handle to the started BeamNG process.
    """

    log_handle: Optional[IO[bytes]] = None
    stdout_target: int | IO[bytes]
    stderr_target: int

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_handle = open(log_path, "ab")
        stdout_target = log_handle
        stderr_target = subprocess.STDOUT
    else:
        stdout_target = subprocess.DEVNULL
        stderr_target = subprocess.DEVNULL

    try:
        for retry in range(10):
            time.sleep(10)
            process = subprocess.Popen(
                cli_command, stdout=stdout_target, stderr=stderr_target, preexec_fn=_pdeathsig_preexec()
            )
            if process.poll() is None:
                return process
            logger.warning(f"⚠️ Attempt {retry + 1} failed with exit code {process.returncode}")
        raise RuntimeError("❌ BeamNG failed to start after 10 retries")
    finally:
        if log_handle is not None:
            log_handle.close()


def _pdeathsig_preexec() -> Optional["Callable[[], None]"]:
    """
    Builds a Popen preexec_fn that ties the launched BeamNG process's lifetime to its parent worker
    process on Linux. Without this, a worker that is force-killed while wedged inside the simulator
    (e.g. reaped on the per-episode deadline) never runs its cleanup, orphaning the BeamNG process and
    leaking its memory — the exact leak the periodic recycle exists to prevent.

    Uses prctl(PR_SET_PDEATHSIG, SIGKILL) so the kernel kills BeamNG when the worker dies. Returns None
    on non-Linux platforms (macOS/WSL host paths), where the feature is unavailable and the parent worker
    cleans BeamNG up explicitly anyway.
    """

    if platform.system() != "Linux":
        return None

    PR_SET_PDEATHSIG = 1

    def _preexec() -> None:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(PR_SET_PDEATHSIG, signal.SIGKILL)

    return _preexec


def terminate_beamng_process(process: Optional[subprocess.Popen], timeout: float = 30.0) -> None:
    """
    Terminates a BeamNG simulator process previously started by ScenarioBridge.

    ScenarioBridge launches BeamNG out-of-band and BeamNGpy connects with launch=False, so BeamNGpy's
    own shutdown does not stop the process, and it must be terminated explicitly, otherwise the orphaned
    process keeps holding (and leaking) memory. No-op when `process` is None (e.g. the macOS path,
    where BeamNG runs on a remote Windows host) or when the process has already exited.

    Args:
        process: The BeamNG process handle, or None if BeamNG was not started locally.
        timeout: Seconds to wait for a graceful shutdown before killing the process.
    """

    if process is None or process.poll() is not None:
        return

    logger.debug("🛑 Terminating BeamNG process (pid %s).", process.pid)
    process.terminate()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        logger.warning("BeamNG process %s ignored SIGTERM; killing it.", process.pid)
        process.kill()
        process.wait(timeout=5)


def create_worker_user_dir(base: PureWindowsPath, worker_index: int) -> PureWindowsPath:
    """Derives a writable per-worker BeamNG user dir from the configured base.

    Every worker (including worker 0) gets its own sibling worker_<i+1> directory next to the base directory.
    To avoid costly downloads of assets, the worker directories get seeded by copying the base directories contents.
    Thus, the base directory must be a read-only template that is never mutated.

    Args:
        base: The base directory containing the BeamNG user dir template.
        worker_index: The index of the worker.

    Returns:
        The path to the BeamNG user dir for the worker.
    """

    # macOS only allows one BeamNG process, so no need to spawn workers.
    if platform.system() == "Darwin":
        return base

    worker_path = base.parent / f"worker_{worker_index + 1}"

    src = to_local_posix(base)
    dst = to_local_posix(worker_path)
    dst.mkdir(parents=True, exist_ok=True)

    # Copy the contents of the base directory to the worker directory to avoid download of assets.
    if src.exists() and src.resolve() != dst.resolve():
        subprocess.run(
            ["rsync", "-a", "--exclude=beamng.log", f"{src}/", f"{dst}/"],
            check=True,
        )
    return worker_path
