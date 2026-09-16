# BeamNG controller library

This repository is a collection of independently installable Python projects.
Each controller has its own dependency set, so controllers with incompatible
third-party requirements can be placed in separate virtual environments.

| Directory | Distribution | Purpose |
| --- | --- | --- |
| `beamng_blackboard` | `beamng-blackboard` | API for synchronizing the scenario-creating process with controllers. |
| `beamng_controllers` | `beamng-controllers` | Shared controller API and lightweight example controllers. |
| `safe_mpd_controller` | `beamng-safe-mpd-controller` | Controller using [Safe-MPD](https://github.com/cps-atlas/safe-mpd). |
| `visionpilot_controller` | `visionpilot_controller` | Controller using [VisionPilot](https://github.com/visionpilot-project/VisionPilot) |

If you don't know where to start, `beamng-controllers` contains two simple
controllers: one that steers the car in random directions, and one that drives
it through a list of waypoints using BeamNG AI.
Please refer to the submodule [README.md](beamng_controllers/README.md).

For a controller environment, first install the shared project, then install
the desired controller project:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e ./beamng_controllers
git clone https://github.com/cps-atlas/safe-mpd.git ../safe-mpd
python -m pip install -e ../safe-mpd
python -m pip install -e ./safe_mpd_controller
```

Safe-MPD must be installed from an editable checkout: its current
non-editable wheel omits planner subpackages required by this controller.

The packages expose `beamng-vehicle-controller` and
`beamng-safe-mpd-controller` command-line programs respectively.
