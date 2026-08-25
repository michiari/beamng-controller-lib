#!/usr/bin/env python3

import argparse
import math
import os
import shutil
from pathlib import Path
import time

import matplotlib
matplotlib.use('TKAgg')

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, set_up_simple_logging

from osm_roads.osm_buildings import load_building_polygons
from beamng_interface.controller.safe_mpd import SafeMPDController
from beamng_interface.controller.vehicle_controller import wait_for_active_vehicles
from beamng_interface.xodr_import import OpenDriveExtendedImporter


DEFAULT_HOST = "localhost"
DEFAULT_PORT = 25252
DEFAULT_VEHICLE_ID = "my_vehicle"
STATUS_POLLING_INTERVAL_S = 1.0
STOP_TIMEOUT_S = 5.0
STOP_SPEED_MPS = 0.1
STOP_HOLD_S = 1.0
TOW_HITCH_TAG = "tow_hitch"
ETK800_HITCH_NODE = "tw"
CARGO_TRAILER_COUPLER_NODE = "t8"


def _node_position(vehicle, node_name):
    """Return a named vehicle node's current world-space position."""
    node_info = vehicle.get_node_info([node_name])
    if not node_info or node_info[0].get("pos") is None:
        raise RuntimeError(
            f"Vehicle {vehicle.vid!r} does not have the expected node "
            f"{node_name!r}."
        )
    return node_info[0]["pos"]


def spawn_vehicle_and_trailer(scenario, vehicle_spawn_pos, vehicle_rot_quat, vehicle, trailer):
    """Align the cargo trailer receiver with the ETK800 hitch and attach it."""
    scenario.add_vehicle(vehicle, pos=vehicle_spawn_pos, rot_quat=vehicle_rot_quat, cling=True)
    scenario.add_vehicle(trailer, pos=vehicle_spawn_pos, rot_quat=vehicle_rot_quat, cling=True)

    hitch_pos = _node_position(vehicle, ETK800_HITCH_NODE)
    coupler_pos = _node_position(trailer, CARGO_TRAILER_COUPLER_NODE)

    trailer.sensors.poll()
    trailer_pos = trailer.state["pos"]
    aligned_pos = tuple(
        trailer_pos[axis] + hitch_pos[axis] - coupler_pos[axis]
        for axis in range(3)
    )
    trailer.teleport(pos=aligned_pos, reset=True)

    # Activating the towing vehicle's coupler makes it latch onto the trailer's
    # matching `tow_hitch` tag as soon as simulation resumes.
    vehicle.couplers.attach(TOW_HITCH_TAG)


def stop_vehicle(vehicle):
    """Leave a vehicle stationary after this controller disconnects."""
    # ``stopping`` is an asynchronous AI mode. In particular, a towing vehicle
    # can keep rolling long enough that disconnecting immediately afterwards
    # leaves its previous AI script in control. Disable AI first, then leave
    # explicit, persistent brake inputs behind in the simulator.
    print(f"Trying to stop vehicle {vehicle.vid}...")

    deadline = time.monotonic() + STOP_TIMEOUT_S
    stationary_since = None

    while True:
        # Reapply both commands while the vehicle decelerates. This prevents a
        # still-running ScriptAI update from winning a one-off shutdown race.
        vehicle.ai.set_mode("disabled")
        vehicle.control(
            throttle=0.0,
            steering=0.0,
            brake=1.0,
            parkingbrake=1.0,
        )
        vehicle.sensors.poll()
        velocity = vehicle.state.get("vel", (0.0, 0.0, 0.0))
        speed = math.sqrt(sum(component * component for component in velocity))
        if speed <= STOP_SPEED_MPS:
            stationary_since = stationary_since or time.monotonic()
            if time.monotonic() - stationary_since >= STOP_HOLD_S:
                print(f"Vehicle {vehicle.vid} stopped ({speed:.3f} m/s).")
                return
        else:
            stationary_since = None
        if time.monotonic() >= deadline:
            print(
                f"Warning: vehicle {vehicle.vid} is still moving at "
                f"{speed:.3f} m/s; leaving its parking brake applied."
            )
            return
        time.sleep(0.1)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Connect to an existing BeamNG scenario and control the two "
            "vehicles created by process A."
        )
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--home",
        default=os.environ.get("BEAMNG_TECH_HOME"),
        type=Path,
        help="BeamNG.tech installation path. Defaults to BEAMNG_TECH_HOME.",
    )
    parser.add_argument(
        "--user",
        default=os.environ.get("BEAMNG_TECH_USER"),
        type=Path,
        help="BeamNG.tech user path. Defaults to BEAMNG_TECH_USER.",
    )
    parser.add_argument("--wait-for-beamng", action="store_true", help="Wait for user input before connecting to BeamNG.tech.")
    parser.add_argument("--vehicle-id", default=DEFAULT_VEHICLE_ID)
    parser.add_argument(
        "--focus-vehicle",
        action="store_true",
        help="Focus the simulator camera on the vehicle after connecting.",
    )
    parser.add_argument(
        "--respawn-vehicle",
        action="store_true",
        help="Respawn the vehicle at the start of the route after connecting.",
    )
    parser.add_argument(
        "--trailer",
        action="store_true",
        help="Use a trailer vehicle model instead of a car.",
    )
    parser.add_argument(
        "--connect-timeout-s",
        type=float,
        default=10.0,
        help="Seconds to wait for vehicle to appear.",
    )
    parser.add_argument(
        "--xodr-file",
        type=Path,
        required=True,
        help="Existing OpenDRIVE file.",
    )
    
    SafeMPDController.build_parser(parser)

    return parser


def build_controller(vehicle, args):
    # TODO: see if we can avoid creating a Scenario here
    scenario = Scenario("tech_ground", "safe_mpd_xodr_import")
    imported_roads = OpenDriveExtendedImporter.import_xodr(
        str(args.xodr_file),
        scenario,
        {
            "material": "road_asphalt_light",
            "default_width": args.lane_width * 2.0,
            "drivability": 1,
            "texture_length": 50.0,
            "surface_type": "decal",
            "min_elevation": 0.0,
        },
    )

    if args.buildings is not None:
        building_polygons, building_offsets = load_building_polygons(args.buildings)

    return SafeMPDController(
        vehicle,
        xodr_roads=imported_roads,
        spawn_pos=args.spawn_pos,
        goal_pos=args.goal_pos,
        waypoint_spacing_m=args.waypoint_spacing,
        max_speed=args.max_speed,
        dynamics=args.dynamics,
        trailer=args.trailer,
        seed=args.seed,
        lane_width=args.lane_width,
        horizon=args.horizon,
        samples=args.samples,
        diffusion_steps=args.diffusion_steps,
        debug=args.debug,
        time_step=args.time_step,
        buildings=(building_polygons, building_offsets) if args.buildings is not None else None
    )


def run_vehicle_controller(
    beamng_host,
    beamng_port,
    beamng_home,
    beamng_user,
    vehicle_id,
    focus_vehicle,
    respawn_vehicle,
    use_trailer,
    connect_timeout_s,
    controller_args,
):
    set_up_simple_logging()

    controller = build_controller(None, controller_args)
    controller.plan_entire_route()

    beamng_client = BeamNGpy(host=beamng_host, port=beamng_port, home=beamng_home, user=beamng_user)
    vehicle = None
    trailer = None
    debug_trajectory_line = None

    try:
        if args.wait_for_beamng:
            input("Press Enter to connect to BeamNG.tech...")
        beamng_client.open(launch=False)
        # Wait before connecting the scenario. ``get_current()`` already
        # connects every vehicle, so connecting a second object returned by
        # ``vehicles.get_current()`` leaks one socket per vehicle.
        wait_for_active_vehicles(
            beamng_client,
            [vehicle_id],
            connect_timeout_s,
        )
        running_scenario = beamng_client.scenario.get_current()
        # A scenario obtained with get_current() is connected but BeamNGpy does
        # not register it for cleanup. Register it so disconnect() closes all
        # vehicle sockets, including the trailer.
        beamng_client._scenario = running_scenario
        print(f"Controller connected to scenario: {running_scenario.name}")

        vehicle = running_scenario.get_vehicle(vehicle_id)
        if vehicle is None:
            raise RuntimeError(f"Vehicle {vehicle_id!r} disappeared while connecting.")

        x, y, theta = controller.get_true_spawn_pos()
        pos_triple = (x, y, args.spawn_pos[2])
        rot_quat = angle_to_quat((0, 0, -math.degrees(theta) - 90.0))
        # Keep newly spawned vehicles stationary while their coupling nodes are
        # aligned. Otherwise spawning both at the same origin causes a collision.
        beamng_client.pause()
        if respawn_vehicle:
            vehicle.disconnect()
            running_scenario.remove_vehicle(vehicle)
            if use_trailer:
                trailer = running_scenario.get_vehicle(vehicle_id + "_trailer")
                if trailer is not None:
                    trailer.disconnect()
                    running_scenario.remove_vehicle(trailer)

                pc_rel_path = Path("vehicles") / "etk800" / "hitch.pc"
                pc_path = Path(beamng_user) / "current" / pc_rel_path
                shutil.copy("hitch.pc", pc_path)
                vehicle = Vehicle(vehicle_id, model="etk800", license="AI", part_config=str(pc_rel_path))
                trailer = Vehicle(vehicle_id + "_trailer", model="boxutility", license="AI")
                spawn_vehicle_and_trailer(running_scenario, pos_triple, rot_quat, vehicle, trailer)

            else:
                vehicle = Vehicle(vehicle_id, model="etk800", license="AI")
                running_scenario.add_vehicle(vehicle, pos=pos_triple, rot_quat=rot_quat, cling=True)
        else:
            vehicle.teleport(pos=pos_triple, rot_quat=rot_quat, reset=True)
        
        if focus_vehicle:
            vehicle.focus()

        controller.set_vehicle(vehicle)
        debug_trajectory_line = controller.add_debug_trajectory_line(beamng_client)
        controller.control_entire_route()

        beamng_client.resume()

        while True:
            time.sleep(STATUS_POLLING_INTERVAL_S)
            vehicle.sensors.poll()
            current_pos = vehicle.state["pos"]
            if controller._comp_waypoints(current_pos, controller_args.goal_pos) < 1.0:
                print("Vehicle has reached the goal position.")
                time.sleep(STATUS_POLLING_INTERVAL_S)
                break

    finally:
        if debug_trajectory_line is not None:
            beamng_client.debug.remove_polyline(debug_trajectory_line)
        if vehicle is not None:
            try:
                stop_vehicle(vehicle)
            except Exception as exc:
                print(f"Could not stop {vehicle.vid!r}: {exc}")
        beamng_client.disconnect()


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()

    set_up_simple_logging()

    run_vehicle_controller(
        beamng_host=args.host,
        beamng_port=args.port,
        beamng_home=args.home,
        beamng_user=args.user,
        vehicle_id=args.vehicle_id,
        respawn_vehicle=args.respawn_vehicle,
        focus_vehicle=args.focus_vehicle,
        use_trailer=args.trailer,
        connect_timeout_s=args.connect_timeout_s,
        controller_args=args,
    )
