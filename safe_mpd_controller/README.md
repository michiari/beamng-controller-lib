# beamng-safe-mpd-controller

A BeamNG.tech controller that uses the
[Safe-MPD planner](https://github.com/cps-atlas/safe-mpd).

Safe-MPD currently publishes an incomplete non-editable wheel: its packaging
configuration includes only the top-level `mbd` package and omits its `envs`,
`planners`, and `robots` subpackages. Install a source checkout in editable
mode before installing this controller.

Create a dedicated environment, install the shared package, then install
Safe-MPD and this controller:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e ../beamng_controllers
git clone https://github.com/cps-atlas/safe-mpd.git ../safe-mpd
python -m pip install -e ../safe-mpd
python -m pip install -e .
```
