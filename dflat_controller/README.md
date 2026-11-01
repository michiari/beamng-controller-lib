# beamng-dflat-controller

A BeamNG.tech controller using an existing local DFLAT-AIT checkout. It imports
OpenDRIVE roads, builds wall and building obstacles, plans a complete route,
and sends the resulting timed path to BeamNG ScriptAI. The connection,
blackboard synchronization, teleport/respawn, and shutdown behavior are shared
with `safe_mpd_controller`.

Install from this repository's root:

```bash
python -m venv .venv-dflat-controller
. .venv-dflat-controller/bin/activate
python -m pip install -e ./beamng_blackboard -e ./beamng_controllers \
    -e ./safe_mpd_controller -e ./dflat_controller
```

The Safe-MPD controller package supplies shared geometry and connection code;
the `mbd` planner itself is not needed. DFLAT-AIT stays in its own checkout and
Python environment; install its dependencies using that checkout's setup
instructions. No DFLAT-AIT source is copied or installed by this project.

Example, connecting to an existing scenario:

```bash
beamng-dflat-controller \
    --dflat-root /path/to/DFLAT-AIT \
    --dflat-python /path/to/DFLAT-AIT/.venv/bin/python \
    --xodr-file /path/to/roads.xodr --lane-width 3.6 \
    --vehicle-id ego_vehicle \
    --spawn-pos 5 0 0 --goal-pos 45 0 0 \
    --model car --samples 192 --diffusion-steps 30
```

Use coordinates on your imported roads. `--spawn-pos` and `--goal-pos` are
BeamNG XYZ positions; route direction determines endpoint headings. Road width
is twice `--lane-width`, matching the Safe-MPD controller. `--buildings` accepts
the same building polygon JSON. Run `python -m dflat_controller.dflat_controller
--help` for connection, respawn, focus, debugging, and planner options.

`DFLAT_AIT_ROOT` can supply the checkout path. `--dflat-python` defaults to the
current interpreter, so a separate environment is optional. Planning uses
temporary JSON files and a subprocess, with logs streamed to the terminal;
`--planning-timeout-s` limits the whole subprocess (default 300 seconds).
Planning finishes before connecting to BeamNG. Failed searches, collisions,
goal errors above 1 metre, and heading errors above 15 degrees abort playback.
Both DFLAT `car` and `acc` models are supported. Trailers are rejected.

To save planning pictures, pass `--debug-output-dir ./planning-debug` on the
command line, or `debug_output_dir="./planning-debug"` to `DFLATController`.
This uses DFLAT-AIT's own `run_dflat.py` renderer to save a trajectory PNG,
a GIF showing diffusion denoising followed by tracking, and a trajectory CSV.
Each plan creates a separate `plan-*` directory below the requested directory;
the terminal output lists the saved file paths. Relative paths resolve against
the controller process's working directory. Output is disabled by default.

Pictures are saved before the collision/goal checks, so a rejected trajectory
can still be inspected. A failed search with no trajectory produces no pictures.
Rendering counts toward `--planning-timeout-s`; increase it for long animations.
The existing `--debug` flag continues to open the road-geometry plot separately.

Match the planner to the simulated vehicle using `--wheelbase`,
`--front-overhang`, `--rear-overhang`, `--vehicle-width`,
`--steering-limit-deg`, and `--max-speed`. Defaults are DFLAT's 2.70 m wheelbase,
0.90/1.10 m front/rear overhangs, 1.90 m width, 35 degree steering limit, and
3 m/s speed limit. `--time-step` defaults to 0.25 seconds.

DFLAT plans at the rear axle. Set `--rear-axle-offset-m` to the forward distance
from the rear axle to the BeamNG vehicle/ScriptAI reference point (default 0).
The controller converts both endpoint poses and the returned path using this
offset. ScriptAI follows timestamped positions with terrain clinging; it does
not execute DFLAT's throttle/steering commands or enforce its heading states.
Validate tracking, especially reversing maneuvers, with your vehicle in BeamNG.

Programmatic use:

```python
from dflat_controller import DFLATController

controller = DFLATController(
    vehicle, imported_roads, lane_width=3.6,
    spawn_pos=(5, 0, 0), goal_pos=(45, 0, 0),
    dflat_root="/path/to/DFLAT-AIT",
    dflat_python="/path/to/DFLAT-AIT/.venv/bin/python",
)
controller.plan_entire_route()
controller.control_entire_route()
```

Run focused tests from the repository root:

```bash
.venv/bin/python -m unittest dflat_controller.test_dflat
# Include the real planner smoke test for both models:
DFLAT_AIT_ROOT=/home/chiarim/Repos/DFLAT-AIT \
DFLAT_PYTHON=/home/chiarim/Repos/DFLAT-AIT/.venv/bin/python \
    .venv/bin/python -m unittest dflat_controller.test_dflat
```
