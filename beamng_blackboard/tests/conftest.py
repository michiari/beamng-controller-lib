# contents of test_append.py
import pytest
import os
import yaml
import subprocess
import uuid

from pathlib import Path, PurePosixPath, PureWindowsPath
from beamngpy import set_up_simple_logging

from beamng import BeamNGWrapper, BeamNGConfigs

@pytest.fixture
def system_is():
    import platform
    return platform.uname().release.lower()

@pytest.fixture
def test_conf():
    # from importlib import resources
    # import tests as this_package
    # text = resources.files(this_package).joinpath("data", "test_utils_scenario1.csv").read_text(encoding="utf-8")
    tests_folder = Path(__file__).parent
    test_conf_file = tests_folder / "beamng.yaml"
    return yaml.safe_load(test_conf_file.open(encoding="utf-8"))


def _retrieve_windows_temp_folder():
    return PureWindowsPath(str(subprocess.run("cmd.exe /c \"echo %TEMP%\" 2> /dev/null", shell=True, capture_output=True, text=True)))

@pytest.fixture
def beamng_home_path(system_is, test_conf):
    if "wsl" in system_is:
        beamng_home = test_conf['beamng']['home'] if 'home' in test_conf['beamng'] else os.environ.get("BEAMNG_TECH_HOME")
        assert beamng_home is not None, "Missing beamng_home"
        return PureWindowsPath(beamng_home)

    raise RuntimeError("Cannot handle non WSL systems at the moment")


@pytest.fixture
def beamng_temp_user_base(system_is, test_conf):
    if "wsl" in system_is:
        # Note to make this work we need to get the temp folder on Windows!
        beamng_temp_folder = test_conf['beamng']['temp_user'] if 'temp_user' in test_conf['beamng'] else _retrieve_windows_temp_folder()
        assert beamng_temp_folder is not None, "Missing beamng_temp_folder"
        # return PureWindowsPath("C:\\BeamNG\\BeamNG.tech.v0.38.3.0")
        return PureWindowsPath(beamng_temp_folder)

    raise RuntimeError("Cannot handle non WSL systems at the moment")


@pytest.fixture
def beamng_temporary_user_folder(system_is, beamng_temp_user_base: Path):  
    """
    Create a fresh temporary folder inside the beamng_base_user folder.
    """
    return beamng_temp_user_base / str(uuid.uuid4())

@pytest.fixture
def beamng_port(system_is, test_conf):
    if "wsl" in system_is:
        # Note to make this work we need to get the temp folder on Windows!
        beamng_port = test_conf['beamng']['port'] if 'port' in test_conf['beamng'] else 25252
        assert beamng_port is not None, "Missing beamng_port"
        # return PureWindowsPath("C:\\BeamNG\\BeamNG.tech.v0.38.3.0")
        return int(beamng_port)

    raise RuntimeError("Cannot handle non WSL systems at the moment")


@pytest.fixture
def running_beamng(beamng_home_path, beamng_temporary_user_folder, beamng_port):
    set_up_simple_logging()

    beamng_configs = BeamNGConfigs(beamng_home_path, beamng_temporary_user_folder, beamng_port=beamng_port, fps=30, step_seconds=30, headless=False)

    print("Summary of BeamNG Config")
    print(f"{beamng_configs}")

    bng_wrapper = BeamNGWrapper(beamng_configs)
    beamng = bng_wrapper.setup_environment()

    # Pass it to the tests
    yield beamng

    # Ensure this is closed when the session ends
    bng_wrapper.close()
