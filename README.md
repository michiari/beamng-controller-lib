# BeamNG controller library

This repository is a collection of independently installable Python projects.
Each controller has its own dependency set, so controllers with incompatible
third-party requirements can be placed in separate virtual environments.

| Directory | Distribution | Purpose |
| --- | --- | --- |
| `beamng_controllers` | `beamng-controllers` | Shared controller API and lightweight example controllers. |
| `safe_mpd_controller` | `beamng-safe-mpd-controller` | Controller using [Safe-MPD](https://github.com/cps-atlas/safe-mpd). |

For a controller environment, first install the shared project, then install
the desired controller project:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e ./beamng_controllers
python -m pip install -e ./safe_mpd_controller
```

The packages expose `beamng-vehicle-controller` and
`beamng-safe-mpd-controller` command-line programs respectively.
