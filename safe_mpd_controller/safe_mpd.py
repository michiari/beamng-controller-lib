"""Convert BeamNG road geometry into obstacle walls for Safe-MPD.

The roads themselves are free space.  This module unions all road corridors and
turns the boundary of that union into the obstacle format used by
``mbd.envs.env.Env``:

* rectangles: ``[center_x, center_y, width, height, angle_radians]``

Unioning the corridors is important at junctions: it removes the boundaries of
overlapping road polygons, so the generated walls do not close an intersection.
"""

from __future__ import annotations
from argparse import ArgumentParser
import math
from typing import Any

from beamngpy.types import Float3

from mbd.envs import get_env
from mbd.planners.mbd_planner import MBDConfig, run_diffusion, clear_jit_cache

from beamng_controllers.controller_wrapper import ControllerWrapper
from .obstacle_builder import RoadBoundaryObstacleBuilder, RoadBoundaryObstacles


class SafeMPDController(ControllerWrapper):
    """A controller that uses Safe-MPD to generate a navigation environment."""

    def __init__(
            self,
            vehicle,
            xodr_roads,
            lane_width: float,
            spawn_pos: Float3,
            goal_pos: Float3,
            waypoint_spacing_m: float = 25.0,
            time_step: float = 0.25,
            max_speed: float = 10.0,
            trailer: bool = False,
            seed: int = 42,
            dynamics: str = "tt2d",
            samples: int = 1000,  # number of samples
            horizon: int = 500,  # horizon
            diffusion_steps: int = 100, # number of diffusion steps
            buildings: Any = None,
            debug: bool = False
        ):
        self.vehicle = vehicle
        self.trajectory_states = None
        self.ai_script = None

        obstacle_builder = RoadBoundaryObstacleBuilder(
            xodr_roads=xodr_roads,
            road_width_m=lane_width * 2.0,
            building_polygons=buildings
        )
        self.base_env_config = obstacle_builder.create_mbd_env()

        self.config = MBDConfig(
            seed=seed,
            env_name=dynamics,
            case="navigation",
            Nsample=samples,
            Hsample=horizon,
            Ndiffuse=diffusion_steps,
            dt=time_step,
            render=debug,
            show_animation=False,
            save_animation=False,
            save_denoising_animation=False,
            verbose=debug,
            num_trailers=1 if trailer else 0,
            #l2=1.0, # trailer length
            # lh=1.0, # hitch length
            #trailer_width=0.5,
            v_max=max_speed
        )

        obstacles = obstacle_builder.build()
        print(f"Road centerlines: {len(obstacle_builder.road_centerlines)}; width: {lane_width * 2.0:.2f} m")
        print(
            "Boundary obstacles: "
            f"{len(obstacles.rectangles)} rectangles"
        )

        # obstacle_builder.plot_debug([spawn_pos, goal_pos], show=True)

        self.waypoints = obstacle_builder.waypoints_between(spawn_pos, goal_pos, spacing_m=waypoint_spacing_m)
        print(f"Generated waypoints: {len(self.waypoints)} points, {self.waypoints[0]} -> {self.waypoints[-1]}")
        self.first_waypoint, self.last_waypoint = self.waypoints[0], self.waypoints[-1]

        if debug:
            obstacle_builder.plot_debug(self.waypoints, show=True)


    def _create_robot_env(self, init_pos: Float3, goal_pos: Float3):
        """Create the MBD robot environment around the configured base Env."""
        self.robot_env = get_env(
            self.config.env_name,
            case=self.config.case,
            env_config=self.base_env_config,
            dt=self.config.dt,
            H=self.config.Hsample,
            motion_preference=self.config.motion_preference,
            collision_penalty=self.config.collision_penalty,
            enable_shielded_rollout_collision=self.config.enable_shielded_rollout_collision,
            hitch_penalty=self.config.hitch_penalty,
            enable_shielded_rollout_hitch=self.config.enable_shielded_rollout_hitch,
            enable_projection=self.config.enable_projection,
            enable_guidance=self.config.enable_guidance,
            reward_threshold=self.config.reward_threshold,
            ref_reward_threshold=self.config.ref_reward_threshold,
            max_w_theta=self.config.max_w_theta,
            hitch_angle_weight=self.config.hitch_angle_weight,
            l1=self.config.l1,
            l2=self.config.l2,
            lh=self.config.lh,
            lf1=self.config.lf1,
            lr=self.config.lr,
            lf2=self.config.lf2,
            lr2=self.config.lr2,
            tractor_width=self.config.tractor_width,
            trailer_width=self.config.trailer_width,
            v_max=self.config.v_max,
            delta_max_deg=self.config.delta_max_deg,
            a_max=self.config.a_max,
            omega_max=self.config.omega_max,
            d_thr_factor=self.config.d_thr_factor,
            k_switch=self.config.k_switch,
            steering_weight=self.config.steering_weight,
            preference_penalty_weight=self.config.preference_penalty_weight,
            heading_reward_weight=self.config.heading_reward_weight,
            terminal_reward_threshold=self.config.terminal_reward_threshold,
            terminal_reward_weight=self.config.terminal_reward_weight,
            ref_pos_weight=self.config.ref_pos_weight,
            ref_theta1_weight=self.config.ref_theta1_weight,
            ref_theta2_weight=self.config.ref_theta2_weight,
            num_trailers=self.config.num_trailers,
        )
        self.robot_env.set_init_pos(
            x=init_pos[0],
            y=init_pos[1],
            theta1=init_pos[2],
            theta2=init_pos[2],
        )
        self.robot_env.set_goal_pos(
            x=goal_pos[0],
            y=goal_pos[1],
            theta1=goal_pos[2],
            theta2=goal_pos[2],
        )


    def set_vehicle(self, vehicle):
        """Set the vehicle to be controlled by this Safe-MPD controller."""
        self.vehicle = vehicle


    def get_true_spawn_pos(self) -> Float3:
        return self.first_waypoint
    

    def get_true_goal_pos(self) -> Float3:
        return self.last_waypoint


    def _set_next_init_goal_pos(self):
        if len(self.waypoints) < 2:
            raise ValueError("Not enough waypoints to set next init and goal positions.")
        next_init = self.waypoints.pop(0)
        next_goal = self.waypoints[0]
        self._create_robot_env(init_pos=next_init, goal_pos=next_goal)
        print(f"Set next init position: {next_init}, next goal position: {next_goal}")
        clear_jit_cache()


    def next_control(self):
        if self.vehicle is None:
            raise ValueError("Vehicle must be set before calling next_control.")

        self._set_next_init_goal_pos()
        self._plan()
        self.vehicle.ai.set_script(self._trajectory_to_ai_script())


    def control_entire_route(self):
        if self.vehicle is None:
            raise ValueError("Vehicle must be set before calling control_entire_route.")

        if self.ai_script is None:
            self.plan_entire_route()
        self.vehicle.ai.set_script(self.ai_script)


    def _plan(self):
        print("Running Safe-MPD planner...")
        rew_final, _actions, self.trajectory_states, timing_info = run_diffusion(
            args=self.config,
            env=self.robot_env,
        )
        print("MPD planning completed.")
        print(f"Number of trajectory states: {len(self.trajectory_states)}")
        print(f"Final reward: {float(rew_final):.3f}")
        print(f"Pure diffusion time: {timing_info['pure_diffusion_time']:.2f}s")
        print(f"Total time: {timing_info['total_time']:.2f}s")


    def plan_entire_route(self):
        self.ai_script = []
        while len(self.waypoints) > 1:
            self._set_next_init_goal_pos()
            self._plan()
            start_t = self.ai_script[-1]["t"] + self.config.dt if self.ai_script else 0.0
            self.ai_script.extend(self._trajectory_to_ai_script(start_t=start_t))

            assert self.trajectory_states is not None
            if self._comp_waypoints(self.trajectory_states[-1], self.waypoints[0]) > 1.0:
                print(f"Warning: Last trajectory state {self.trajectory_states[-1].tolist()} "
                      f"does not match the next waypoint {list(self.waypoints[0])}.")


    def _trajectory_to_ai_script(self, start_t=0.0):
        if self.trajectory_states is None:
            raise ValueError("Trajectory states are not available. Run _plan() first.")
        ai_script = [
            {
                "x": self.trajectory_states[i, 0].item(),
                "y": self.trajectory_states[i, 1].item(),
                "z": 0.0, # TODO: use terrain height
                "t": start_t + self.config.dt * i,
            }
            for i in range(len(self.trajectory_states))
        ]
        return ai_script


    def _comp_waypoints(self, waypoint1: Float3, waypoint2: Float3) -> float:
        """Compute the Euclidean distance between two waypoints."""
        return math.sqrt(
            (waypoint1[0] - waypoint2[0]) ** 2 +
            (waypoint1[1] - waypoint2[1]) ** 2
        )


    def add_debug_trajectory_line(self, beamng):
        if self.ai_script is None:
            raise ValueError("AI script is not available. Run plan_entire_route() first.")
        return beamng.debug.add_polyline(
            [(n["x"], n["y"], n["z"]) for n in self.ai_script],
            (1, 0, 0, 1),
            cling=True,
            offset=0.1
        )
        

    @staticmethod
    def build_parser(parser: ArgumentParser) -> None:
        """Add command-line arguments accepted by ``SafeMPDController``."""

        parser.add_argument(
            "--lane-width",
            type=float,
            required=True,
            help="Lane width in metres",
        )
        parser.add_argument(
            "--spawn-pos",
            type=float,
            nargs=3,
            required=False,
            metavar=("X", "Y", "Z"),
            help="Initial vehicle position",
        )
        parser.add_argument(
            "--goal-pos",
            type=float,
            nargs=3,
            required=True,
            metavar=("X", "Y", "Z"),
            help="Goal vehicle position",
        )
        parser.add_argument(
            "--waypoint-spacing",
            type=float,
            default=math.inf,
            help="Waypoint spacing in metres. If not set, does not create intermediate waypoints.)"
        )
        parser.add_argument(
            "--time-step",
            type=float,
            default=0.25,
            help="Simulation time step in seconds (default: %(default)s)"
        )
        parser.add_argument(
            "--max-speed",
            type=float,
            default=10.0,
            help="Maximum vehicle speed in m/s (default: %(default)s)"
        )
        parser.add_argument(
            "--seed",
            type=int,
            default=42,
            help="Random seed (default: %(default)s)",
        )
        parser.add_argument(
            "--dynamics",
            default="tt2d",
            help="MBD dynamics environment (default: %(default)s)",
        )
        parser.add_argument(
            "--samples",
            type=int,
            default=1000,
            help="Number of MBD samples (default: %(default)s)",
        )
        parser.add_argument(
            "--horizon",
            type=int,
            default=500,
            help="MBD planning horizon (default: %(default)s)",
        )
        parser.add_argument(
            "--diffusion-steps",
            type=int,
            default=100,
            help="Number of diffusion steps (default: %(default)s)",
        )
        parser.add_argument(
            "--buildings",
            type=str,
            default=None,
            help="Path to a JSON file containing building polygons (default: %(default)s)",
        )
        parser.add_argument(
            "--debug",
            action="store_true",
            help="Enable MBD rendering and verbose output",
        )


__all__ = [
    "ImportedRoadGeometry",
    "RoadBoundaryObstacleBuilder",
    "RoadBoundaryObstacles",
    "SafeMPDController"
]
