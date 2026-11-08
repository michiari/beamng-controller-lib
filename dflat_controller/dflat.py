"""Plan road routes with DFLAT-AIT and execute them through BeamNG ScriptAI."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

from beamng_controllers.controller_wrapper import ControllerWrapper
from safe_mpd_controller.obstacle_builder import RoadBoundaryObstacleBuilder


class DFLATController(ControllerWrapper):
    def __init__(
        self, vehicle, xodr_roads, lane_width, spawn_pos, goal_pos, *,
        dflat_root, dflat_python=None, model="car", seed=1, samples=192,
        diffusion_steps=30, planning_timeout_s=900.0, time_step=0.25,
        max_speed=3.0, wheelbase=2.70, front_overhang=0.90,
        rear_overhang=1.10, vehicle_width=1.90, steering_limit_deg=35.0,
        rear_axle_offset_m=0.0, buildings=None, trailer=False, debug=False,
        debug_output_dir: str | Path | None = None,
        spawn_heading=None,
        road_height_offset_m=None, script_cling=True,
        boundary_simplify_tolerance_m=0.0,
    ):
        if trailer:
            raise ValueError("DFLAT-AIT supports cars only; trailers are not supported")
        if model not in ("car", "acc"):
            raise ValueError("model must be 'car' or 'acc'")
        for label, position in (("spawn_pos", spawn_pos), ("goal_pos", goal_pos)):
            if np.asarray(position).shape != (3,) or not np.isfinite(position).all():
                raise ValueError(f"{label} must contain three finite XYZ coordinates")
        for label, value in (("samples", samples), ("diffusion_steps", diffusion_steps)):
            if not isinstance(value, int) or value < 1:
                raise ValueError(f"{label} must be a positive integer")

        self.dflat_root = Path(dflat_root).resolve()
        if not (self.dflat_root / "run_dflat.py").is_file():
            raise ValueError(f"DFLAT-AIT run_dflat.py not found in {self.dflat_root}")
        self.dflat_python = str(dflat_python or sys.executable)
        self.planning_timeout_s = RoadBoundaryObstacleBuilder._require_positive(
            "planning_timeout_s", planning_timeout_s
        )
        robot = {
            "dt": time_step, "v_max": max_speed, "l": wheelbase,
            "lf": front_overhang, "lr": rear_overhang, "width": vehicle_width,
            "delta_max_deg": steering_limit_deg,
        }
        robot = {
            key: RoadBoundaryObstacleBuilder._require_positive(key, value)
            for key, value in robot.items()
        }
        if robot["delta_max_deg"] >= 90.0:
            raise ValueError("steering_limit_deg must be less than 90 degrees")
        self.rear_axle_offset_m = float(rear_axle_offset_m)
        if not math.isfinite(self.rear_axle_offset_m):
            raise ValueError("rear_axle_offset_m must be finite")
        self.settings = {
            "planner": {
                "model": model, "seed": seed, "mbd_samples": samples,
                "ndiffuse": diffusion_steps, "time": self.planning_timeout_s,
                "denoise_gif": debug_output_dir is not None,
            },
            "robot": robot,
            "debug_output_dir": (
                None if debug_output_dir is None else str(Path(debug_output_dir).resolve())
            ),
        }
        self.builder = RoadBoundaryObstacleBuilder(
            xodr_roads=xodr_roads,
            road_width_m=None if lane_width is None else lane_width * 2.0,
            building_polygons=buildings,
            simplify_tolerance_m=boundary_simplify_tolerance_m,
        )
        self.waypoints = self.builder.waypoints_between(
            spawn_pos, goal_pos, spacing_m=math.inf
        )
        if spawn_heading is not None:
            if not math.isfinite(spawn_heading):
                raise ValueError("spawn_heading must be finite radians")
            self.waypoints[0] = (*self.waypoints[0][:2], float(spawn_heading))
        self.first_waypoint, self.last_waypoint = self.waypoints[0], self.waypoints[-1]
        if self._comp_waypoints(self.first_waypoint, self.last_waypoint) < 1e-6:
            raise ValueError("spawn and goal must be different positions")
        self.spawn_height = float(spawn_pos[2])
        if road_height_offset_m is not None and not math.isfinite(road_height_offset_m):
            raise ValueError("road_height_offset_m must be finite")
        self.road_height_offset_m = road_height_offset_m
        self.script_cling = script_cling
        self.vehicle = vehicle
        self.ai_script = None
        self.trajectory_states = None
        if debug:
            self.builder.plot_debug(self.waypoints)

    def _rear_axle_pose(self, pose):
        x, y, heading = pose
        return (
            x - self.rear_axle_offset_m * math.cos(heading),
            y - self.rear_axle_offset_m * math.sin(heading),
            heading,
        )

    def plan_entire_route(self):
        self.ai_script = None
        self.trajectory_states = None
        with tempfile.TemporaryDirectory(prefix="beamng-dflat-") as directory:
            work = Path(directory)
            self.builder.export_dflat_map(
                work / "map.json", start=self._rear_axle_pose(self.first_waypoint),
                goal=self._rear_axle_pose(self.last_waypoint),
            )
            settings_path = work / "settings.json"
            settings_path.write_text(json.dumps(self.settings, allow_nan=False), encoding="utf-8")
            output = work / "trajectory.json"
            subprocess.run(
                [self.dflat_python, str(Path(__file__).with_name("planner.py")),
                 str(self.dflat_root), str(work / "map.json"), str(settings_path), str(output)],
                check=True, timeout=self.planning_timeout_s,
                env={**os.environ, "MPLBACKEND": "Agg"},
            )
            result = json.loads(output.read_text(encoding="utf-8"))
        states = np.asarray(result["states"], dtype=float)
        dt = RoadBoundaryObstacleBuilder._require_positive("trajectory dt", result["dt"])
        if (states.ndim != 2 or states.shape[0] < 3 or states.shape[1] < 3
                or not np.isfinite(states).all()):
            raise ValueError("DFLAT must return at least three finite (x, y, heading) states")
        self.ai_script = [
            {
                "x": float(x + self.rear_axle_offset_m * math.cos(heading)),
                "y": float(y + self.rear_axle_offset_m * math.sin(heading)),
                "z": self.spawn_height, "t": index * dt,
            }
            for index, (x, y, heading) in enumerate(states[:, :3])
        ]
        if self.road_height_offset_m is not None:
            from shapely.geometry import LineString, Point

            lines = [LineString([node[:3] for node in road.nodes])
                     for road in self.builder.xodr_roads]
            for node in self.ai_script:
                point = Point(node["x"], node["y"])
                line = min(lines, key=lambda candidate: candidate.distance(point))
                node["z"] = line.interpolate(line.project(point)).z + self.road_height_offset_m
        self.trajectory_states = states
        return self.ai_script

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def get_true_spawn_pos(self):
        return self.first_waypoint

    def get_true_goal_pos(self):
        return self.last_waypoint

    def control_entire_route(self):
        if self.vehicle is None:
            raise ValueError("Vehicle must be set before calling control_entire_route")
        if self.ai_script is None:
            self.plan_entire_route()
        self.vehicle.ai.set_script(self.ai_script, cling=self.script_cling)

    def next_control(self):
        self.control_entire_route()

    @staticmethod
    def _comp_waypoints(waypoint1, waypoint2):
        return math.hypot(waypoint1[0] - waypoint2[0], waypoint1[1] - waypoint2[1])

    def add_debug_trajectory_line(self, beamng):
        if self.ai_script is None:
            raise ValueError("Run plan_entire_route before drawing the trajectory")
        return beamng.debug.add_polyline(
            [(node["x"], node["y"], node["z"]) for node in self.ai_script],
            (1, 0, 0, 1), cling=True, offset=0.1,
        )

    @staticmethod
    def build_parser(parser):
        root = os.environ.get("DFLAT_AIT_ROOT")
        parser.add_argument("--dflat-root", type=Path, default=root, required=root is None,
                            help="Local DFLAT-AIT checkout (or DFLAT_AIT_ROOT)")
        parser.add_argument("--dflat-python", default=sys.executable,
                            help="Python executable with DFLAT-AIT dependencies")
        parser.add_argument("--lane-width", type=float,
                            help="Half road width override in metres; required with --xodr-file")
        parser.add_argument("--spawn-pos", type=float, nargs=3, metavar=("X", "Y", "Z"),
                            help="Start XYZ; defaults to the vehicle's current pose with --beamng-roads")
        parser.add_argument("--goal-pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
        parser.add_argument("--model", choices=("car", "acc"), default="car")
        parser.add_argument("--seed", type=int, default=1)
        parser.add_argument("--samples", type=int, default=192)
        parser.add_argument("--diffusion-steps", type=int, default=30)
        parser.add_argument("--planning-timeout-s", type=float, default=900.0)
        parser.add_argument("--time-step", type=float, default=0.25)
        parser.add_argument("--max-speed", type=float, default=3.0)
        parser.add_argument("--wheelbase", type=float, default=2.70)
        parser.add_argument("--front-overhang", type=float, default=0.90)
        parser.add_argument("--rear-overhang", type=float, default=1.10)
        parser.add_argument("--vehicle-width", type=float, default=1.90)
        parser.add_argument("--steering-limit-deg", type=float, default=35.0)
        parser.add_argument("--rear-axle-offset-m", type=float, default=0.0,
                            help="Forward distance from rear axle to BeamNG vehicle reference point")
        parser.add_argument("--buildings", type=Path)
        parser.add_argument("--debug", action="store_true", help="Plot road geometry before planning")
        parser.add_argument("--debug-output-dir", type=Path,
                            help="Save a trajectory PNG, denoising/tracking GIF, and CSV in a subdirectory per plan")
        parser.add_argument("--endpoint-tolerance-m", type=float, default=0.0,
                            help="Join extracted road endpoints within this XYZ distance for planning")
        parser.add_argument("--goal-runout-m", type=float, default=0.0,
                            help="Planning-only road continuation beyond a terminal goal for vehicle overhang")
        parser.add_argument("--road-height-offset-m", type=float,
                            help="Align script Z to extracted road heights plus this offset")
        parser.add_argument("--no-script-cling", dest="script_cling", action="store_false",
                            help="Disable ScriptAI terrain clinging")
