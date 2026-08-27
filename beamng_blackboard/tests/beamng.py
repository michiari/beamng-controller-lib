import logging
import platform
import subprocess

from dataclasses import dataclass
from typing import Optional
from beamngpy import BeamNGpy
from pathlib import Path, PureWindowsPath, PurePosixPath
from beamng_utils import (
    start_beamng_wsl,
    start_beamng_linux,
    open_beamng_with_retry,
    terminate_beamng_process,
)

logger = logging.getLogger(__name__)

@dataclass(frozen=True)
class BeamNGConfigs():
    beamng_home_path: Path
    # TODO: Rename this in beamng_user_path
    user_path: Path
    beamng_port: int
    fps: int
    step_seconds: int
    headless: bool = False

    def __str__(self) -> str:
        from dataclasses import fields
        string_rep = ""
        for field in fields(BeamNGConfigs):
            string_rep = string_rep + f"{field.name}" + ": " f"{getattr(self, field.name)}" + "\n"
        return string_rep


class BeamNGWrapper():
    #
    # Wrap the startup logic of BeamNG. Based on ScenarioBridge code.
    #
    _BOOT_ATTEMPTS = 5
    _PER_BOOT_TIMEOUT_S = 300.0

    def __init__(
        self,
        beamng_configs: BeamNGConfigs
    ):
        """
        Initializes the BeamNG environment.

        Args:
            beamng_configs: Configuration object containing the BeamNG environment settings.
        """
        self.configs: BeamNGConfigs = beamng_configs
        self.fps = beamng_configs.fps
        self.step_length = max(1, int(round(beamng_configs.step_seconds * self.fps)))

        # Handle to the BeamNG process so close() can terminate it. BeamNGpy only stops processes it
        # launched itself, but ScenarioBridge starts BeamNG out-of-band (see _launch_and_connect).
        self._beamng_process: Optional[subprocess.Popen] = None
        self._closed = False

    def setup_environment(self) -> BeamNGpy:
        last_error: Optional[BaseException] = None
        for boot_attempt in range(1, self._BOOT_ATTEMPTS + 1):
            try:
                self.bng = self._launch_and_connect()
                return self.bng
                # scenario = self._build_scenario()
                # vehicles = self._add_vehicles(scenario)
                # self._load_scenario(bng, scenario)
                # return bng, vehicles
            except Exception as e:
                last_error = e
                logger.warning(
                    "BeamNG boot attempt %d/%d failed: %s. Restarting BeamNG.",
                    boot_attempt,
                    self._BOOT_ATTEMPTS,
                    e,
                )
                terminate_beamng_process(self._beamng_process)
                self._beamng_process = None

        raise RuntimeError(f"❌ BeamNG failed to come up after {self._BOOT_ATTEMPTS} boot attempts: {last_error}")

    def _launch_and_connect(self) -> BeamNGpy:
        """
        Launches the BeamNG simulator process and establishes a single connection to it.

        Depending on the host OS, BeamNG is started from Linux, WSL, or via SSH from macOS. The handshake
        itself is retried internally by open_beamng_with_retry, which bails as soon as the BeamNG process
        dies so a failed boot is detected in seconds rather than after the full handshake timeout. Restarting
        the process on failure is handled one level up by setup_environment's boot-attempt loop.

        Returns:
            The connected BeamNG simulation environment.

        Raises:
            NotImplementedError: If the operating system is not supported or recognized by the method.
        """

        if "wsl" in platform.uname().release.lower():
            host_address, self._beamng_process = start_beamng_wsl(
                runnable_path= PureWindowsPath(self.configs.beamng_home_path) / "Bin64" / "BeamNG.tech.x64.exe",
                user_path=PureWindowsPath(self.configs.user_path),
                port=self.configs.beamng_port,
                headless=self.configs.headless,
            )
            home = PureWindowsPath(self.configs.beamng_home_path).as_posix()
            user = PureWindowsPath(self.configs.user_path).as_posix()
        elif platform.system() == "Linux":
            host_address, self._beamng_process = start_beamng_linux(
                runnable_path=PurePosixPath(self.configs.beamng_home_path) / "BinLinux" / "BeamNG.tech.x64",
                user_path=PurePosixPath(self.configs.user_path),
                port=self.configs.beamng_port,
            )
            home = PurePosixPath(self.configs.beamng_home_path).as_posix()
            user = PurePosixPath(self.configs.user_path).as_posix()
        else:
            raise NotImplementedError(f"ScenarioBridge does not support running BeamNG on {platform.system()}.")

        return open_beamng_with_retry(
            host=host_address,
            port=self.configs.beamng_port,
            home=home, 
            user=user, 
            process=self._beamng_process,
            timeout=self._PER_BOOT_TIMEOUT_S,
        )
    
    def close(self) -> None:
        """
        Cleans up the BeamNG environment, closing the connection and terminating the simulator process.

        Idempotent: a second call is a no-op.
        """

        if self._closed:
            return
        self._closed = True

        self._stop_simulator()

    def _stop_simulator(self) -> None:
        """
        Closes the BeamNG connection and terminates the simulator process.

        Shared by close() and recycle(). Unlike close(), this neither shuts down the thread-pool executor
        nor marks the environment closed, so recycle() can boot a fresh simulator afterwards.
        """

        try:
            self.bng.close()
        except Exception:
            logger.exception("Error closing the BeamNG connection; terminating the process anyway.")
        terminate_beamng_process(self._beamng_process)
        self._beamng_process = None

    def __str__(self):
        return "BeamNG"
