# beamng-dflat-controller

A BeamNG.tech controller using an existing local DFLAT-AIT checkout. It reads
OpenDRIVE or live BeamNG roads, builds wall and building obstacles, plans a complete route,
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

Programmatic callers can pass `lane_width=None` to preserve each imported
road's per-node widths instead of overriding them with one fixed width.

To extract roads from an already loaded BeamNG scenario instead of OpenDRIVE:

```bash
python -m dflat_controller.dflat_controller \
    --dflat-root /home/chiarim/Repos/DFLAT-AIT \
    --dflat-python /home/chiarim/Repos/DFLAT-AIT/.venv/bin/python \
    --beamng-roads --vehicle-id ego_vehicle \
    --spawn-pos 5 0 0 --goal-pos 45 0 0 \
    --debug-output-dir outputs/dflat
```

`--beamng-roads` and `--xodr-file` are mutually exclusive. BeamNG supplies
world-space centerlines, elevations, sampled left/right boundaries and variable
widths. Omit `--lane-width` to use these actual boundaries; supplying it overrides
them with a symmetric corridor of twice that width. Omit `--spawn-pos` to use
the existing vehicle's position and heading without teleporting it. An explicit
start keeps the existing teleport/respawn behavior and uses the route heading.
`--goal-pos` remains a required XYZ coordinate; its heading follows the route.
Vehicle dimensions, steering limits, planner settings and optional building
polygons still come from the command line.
Dense live boundaries are simplified within one millimetre before wall creation
to remove float32 sampling noise; the extracted records retain all samples.

Live extraction connects first and pauses the simulation while planning. It
does not require a blackboard by default. Supply `--run-id` when coordinating
with a scenario process: after planning and installing the script, the controller
marks its vehicle ready and waits for that process to mark the run running.
Playback then resumes the simulation. A planning failure disconnects without
installing a script and leaves the simulation paused for inspection.

Scenario launchers can add `--scenario-managed` with `--beamng-roads --run-id`.
The controller then leaves placement, resuming, stepping and shutdown to the
scenario process; an explicit `--spawn-pos` sets the planning start without
teleporting the vehicle. Respawning is disallowed in this mode.
`--no-script-cling` disables ScriptAI terrain clinging, and
`--road-height-offset-m` aligns script elevations to the extracted centerlines
plus that offset. Without this option, scripts retain the start height.
`--endpoint-tolerance-m` explicitly joins nearby road endpoints in the planning
map, and `--goal-runout-m` adds a planning-only continuation at a terminal goal
to accommodate vehicle overhang. Both margins default to zero and do not
modify the simulator's roads.

The entry point also supports direct file execution, for example
`python /path/to/beamng-controller-lib/dflat_controller/dflat_controller.py ...`.

The shared extractor is `safe_mpd_controller.beamng_roads.extract_beamng_roads`;
it has no planner dependencies. Its records work directly as the builder's
`xodr_roads` input and with either controller using `lane_width=None`.
Each record retains all other BeamNG road fields in `metadata`, including lane,
material and direction fields when supplied. The current planners use geometric
connectivity and do not enforce these metadata fields as traffic rules.
Only DecalRoads with positive drivability are extracted; markings and disabled
roads are filtered out. Mesh roads, buildings and moving obstacles are not
returned by this API. Planning remains two-dimensional, so overlapping bridge
levels cannot be distinguished. Disconnected centerlines are not automatically
joined, and end boundaries are not extended beyond the reported road surface;
choose endpoint positions that leave room for the vehicle's full footprint.

`DFLAT_AIT_ROOT` can supply the checkout path. `--dflat-python` defaults to the
current interpreter, so a separate environment is optional. Planning uses
temporary JSON files and a subprocess, with logs streamed to the terminal;
`--planning-timeout-s` limits the whole subprocess (default 900 seconds).
With OpenDRIVE, planning finishes before connecting to BeamNG. Failed searches, collisions,
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
