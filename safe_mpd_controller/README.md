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

The controller expects an XML [ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
file describing the roads in the scenario.

Example command:
```bash
python -m safe_mpd_controller.safe_mpd_controller --vehicle-id ego_vehicle --xodr-file path/to/xodr/file.xodr --lane-width 3.6 --spawn-pos 22.5 9.0 42 --goal-pos -4.4 35.9 0.0  --samples 4000 --horizon 50 --diffusion-steps 100 --max-speed 20 --trailer --dynamics tt2d --debug
```

For more usage options, consult
```bash
python -m safe_mpd_controller.safe_mpd_controller --help
```

To export the same road-wall and building obstacles for DFLAT-AIT, call
`export_dflat_map` on a configured builder:

```python
from shapely.geometry import LineString
from safe_mpd_controller.obstacle_builder import RoadBoundaryObstacleBuilder

builder = RoadBoundaryObstacleBuilder(
    road_centerlines=[LineString([(0, 0), (50, 0)])],
    road_width_m=8.0,
)
builder.export_dflat_map(
    "road_map.json",
    start=(5.0, 0.0, 0.0),
    goal=(45.0, 0.0, 0.0),
)
```

Start and goal are rear-axle `(x, y, heading)` poses in metres and radians,
using the input geometry's world coordinates. BeamNG XYZ positions need their
height replaced with the intended heading. The optional `name` defaults to
the filename stem; `environment_margin_m` defaults to 1 metre around the map
plus half the wall thickness. The destination directory must exist, and an
existing file is overwritten.

Run the exported map from the DFLAT-AIT checkout:

```bash
python run_dflat.py /absolute/path/to/road_map.json --model car
```
