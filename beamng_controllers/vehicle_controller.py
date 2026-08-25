#!/usr/bin/env python3

import argparse
import os
import time

from beamngpy import BeamNGpy, set_up_simple_logging

from beamng_interface.controller.controller_wrapper import RandomController
from beamng_interface.controller.beamng_ai_controller import BeamNGAIController


DEFAULT_HOST = "localhost"
DEFAULT_PORT = 25252
DEFAULT_VEHICLE_ID = "my_vehicle"
DEFAULT_CONTROLLER_TYPE = "random"
CONTROLLER_TYPES = (DEFAULT_CONTROLLER_TYPE, "beamng-ai")


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
        help="BeamNG.tech installation path. Defaults to BEAMNG_TECH_HOME.",
    )
    parser.add_argument(
        "--user",
        default=os.environ.get("BEAMNG_TECH_USER"),
        help="BeamNG.tech user path. Defaults to BEAMNG_TECH_USER.",
    )
    parser.add_argument("--vehicle-id", default=DEFAULT_VEHICLE_ID)
    parser.add_argument(
        "--iterations",
        type=int,
        default=20,
        help="Number of control commands to send.",
    )
    parser.add_argument(
        "--control-interval-s",
        type=float,
        default=1.0,
        help="Wall-clock seconds between control commands.",
    )
    parser.add_argument(
        "--focus-vehicle",
        action="store_true",
        help="Focus the simulator camera on the vehicle after connecting.",
    )
    parser.add_argument(
        "--connect-timeout-s",
        type=float,
        default=10.0,
        help="Seconds to wait for vehicle to appear.",
    )
    subparsers = parser.add_subparsers(
        title="Controller types",
        dest="controller_type",
        required=True,
    )

    RandomController.build_parser(subparsers.add_parser("random", help="Random controller"))
    BeamNGAIController.build_parser(subparsers.add_parser("beamng-ai", help="BeamNG AI controller"))

    return parser


def validate_args(parser, args):
    if args.connect_timeout_s < 0.0:
        parser.error("--connect-timeout-s must be non-negative")
    if args.iterations < 1:
        parser.error("--iterations must be at least 1")
    if args.control_interval_s < 0.0:
        parser.error("--control-interval-s must be non-negative")


def connect_vehicle(client, active_vehicles, vehicle_id):
    if vehicle_id not in active_vehicles:
        available = ", ".join(sorted(active_vehicles))
        raise RuntimeError(
            f"Vehicle {vehicle_id!r} is not active. Active vehicles: {available}"
        )

    vehicle = active_vehicles[vehicle_id]
    vehicle.connect(client)
    return vehicle


def wait_for_active_vehicles(client, vehicle_ids, timeout_s):
    deadline = time.monotonic() + timeout_s

    while True:
        active_vehicles = client.vehicles.get_current()
        if all(vehicle_id in active_vehicles for vehicle_id in vehicle_ids):
            return active_vehicles

        if time.monotonic() >= deadline:
            available = ", ".join(sorted(active_vehicles))
            expected = ", ".join(vehicle_ids)
            raise RuntimeError(
                f"Timed out waiting for vehicles {expected}. "
                f"Active vehicles: {available}"
            )

        time.sleep(0.25)


def stop_vehicle(vehicle):
    try:
        vehicle.control(throttle=0.0, steering=0.0, brake=1.0)
    except Exception as exc:
        vehicle_id = getattr(vehicle, "vid", "<unknown>")
        print(f"[B] Could not stop {vehicle_id!r}: {exc}")


def build_controller(vehicle, args):
    if args.controller_type == "random":
        return RandomController(
            vehicle,
            min_throttle=args.min_throttle,
            max_throttle=args.max_throttle,
            max_steering=args.max_steering,
        )

    if args.controller_type == "beamng-ai":
        return BeamNGAIController(
            vehicle,
            waypoints=args.waypoints,
            wp_speeds=args.wp_speeds,
            no_of_laps=args.no_of_laps,
            route_speed=args.route_speed,
            route_speed_mode=args.route_speed_mode,
            drive_in_lane=args.drive_in_lane,
            aggression=args.aggression,
            avoid_cars=args.avoid_cars,
        )

    raise ValueError(f"Unsupported controller type: {args.controller_type}")


def run_vehicle_controller(
    beamng_host,
    beamng_port,
    beamng_home,
    beamng_user,
    vehicle_id,
    iterations,
    control_interval_s,
    focus_vehicle,
    connect_timeout_s,
    controller_args,
):
    set_up_simple_logging()

    beamng_client = BeamNGpy(host=beamng_host, port=beamng_port, home=beamng_home, user=beamng_user)
    vehicle = None

    try:
        beamng_client.open(launch=False)
        running_scenario = beamng_client.scenario.get_current()
        print(f"Controller connected to scenario: {running_scenario.name}")

        active_vehicles = wait_for_active_vehicles(
            beamng_client,
            [vehicle_id],
            connect_timeout_s,
        )
        vehicle = connect_vehicle(beamng_client, active_vehicles, vehicle_id)
        
        if focus_vehicle:
            vehicle.focus()

        controller = build_controller(vehicle, controller_args)

        for index in range(iterations):
            controller.next_control()

            print(f"[CLIENT] Command {index + 1:02d}/{iterations}.")

            if control_interval_s:
                time.sleep(control_interval_s)

    finally:
        if vehicle is not None:
            stop_vehicle(vehicle)
        beamng_client.disconnect()


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    set_up_simple_logging()

    run_vehicle_controller(
        beamng_host=args.host,
        beamng_port=args.port,
        beamng_home=args.home,
        beamng_user=args.user,
        vehicle_id=args.vehicle_id,
        iterations=args.iterations,
        control_interval_s=args.control_interval_s,
        focus_vehicle=args.focus_vehicle,
        connect_timeout_s=args.connect_timeout_s,
        controller_args=args,
    )
