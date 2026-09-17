#!/usr/bin/env python3

import argparse
import logging
import os
import time
from pathlib import Path

from beamngpy import BeamNGpy, set_up_simple_logging
import cv2

from beamng_blackboard import BeamNGBlackboard
from beamng_controllers.video_recorder import (
    DEFAULT_VIDEO_RESOLUTION,
    BeamNGVideoRecorder,
)
from visionpilot_controller.visionpilot import DEFAULT_CV_HINT_NUM_LANES, VisionPilotController

logger = logging.getLogger("visionpilot_controller")


DEFAULT_HOST = "localhost"
DEFAULT_PORT = 25252
DEFAULT_VEHICLE_ID = "my_vehicle"
DEFAULT_STEP_SIZE = 10
INITIAL_STEP_SIZE = 10


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
    parser.add_argument("--run-id", default=None, help="Run ID to use for the blackboard")
    parser.add_argument("--vehicle-id", default=DEFAULT_VEHICLE_ID)
    parser.add_argument(
        "--iterations",
        type=int,
        default=-1,
        help="Number of control commands to send. If negative, run until interrupted.",
    )
    parser.add_argument(
        "--step-size",
        type=int,
        default=DEFAULT_STEP_SIZE,
        help="Number of simulation steps to advance per control command.",
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

    parser.add_argument(
        "--cv-hint-num-lanes",
        type=int,
        default=DEFAULT_CV_HINT_NUM_LANES,
        help="Hint for the number of lanes to detect in CV lane detection (default: %(default)s)",
    )
    parser.add_argument(
        "--debug-show-cv-lane-detection-output",
        action="store_true",
        help="Enable debug display for CV lane detection output.",
    )
    parser.add_argument(
        "--debug-cv-lane-detection",
        action="store_true",
        help="Enable debug display for CV lane detection.",
    )
    parser.add_argument(
        "--debug-perspective",
        action="store_true",
        help="Enable debug display for perspective transformation.",
    )

    parser.add_argument(
        "--record-video",
        action="store_true",
        help="Record the vehicle camera to a video.",
    )
    parser.add_argument(
        "--video-path",
        default="visionpilot.mp4",
        type=Path,
        help="Output path for the recorded video (default: %(default)s).",
    )
    parser.add_argument(
        "--video-resolution",
        type=int,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        default=DEFAULT_VIDEO_RESOLUTION,
        help="Video resolution in pixels (default: %(default)s).",
    )
    parser.add_argument(
        "--ffmpeg-path",
        default=None,
        type=Path,
        help="Path to the ffmpeg executable. By default it is auto-detected.",
    )

    return parser


def validate_args(parser, args):
    if args.connect_timeout_s < 0.0:
        parser.error("--connect-timeout-s must be non-negative")
    if any(dimension <= 0 for dimension in args.video_resolution):
        parser.error("--video-resolution WIDTH HEIGHT must be positive")


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
        logger.warning("Could not stop %r: %s", vehicle_id, exc, exc_info=True)


def build_controller(vehicle, args):

    raise ValueError(f"Unsupported controller type: {args.controller_type}")


def run_vehicle_controller(
    beamng_host,
    beamng_port,
    beamng_home,
    beamng_user,
    run_id,
    vehicle_id,
    iterations,
    step_size,
    focus_vehicle,
    connect_timeout_s,
    more_args
):
    set_up_simple_logging()

    beamng_client = BeamNGpy(host=beamng_host, port=beamng_port, home=beamng_home, user=beamng_user)
    vehicle = None

    try:
        beamng_client.open(launch=False)

        blackboard = BeamNGBlackboard(beamng_client)
        if run_id is None:
            snapshot = blackboard.wait_for_ready()
            run_id = snapshot.run_id

        running_scenario = beamng_client.scenario.get_current()
        logger.info("Connected to scenario: %s", running_scenario.name)

        active_vehicles = wait_for_active_vehicles(
            beamng_client,
            [vehicle_id],
            connect_timeout_s,
        )
        vehicle = connect_vehicle(beamng_client, active_vehicles, vehicle_id)
        
        if focus_vehicle:
            vehicle.focus()

        controller = VisionPilotController(
            beamng_client, vehicle,
            cv_hint_num_lanes=more_args.cv_hint_num_lanes,
            debug_show_cv_lane_detection_output=more_args.debug_show_cv_lane_detection_output,
            debug_cv_lane_detection=more_args.debug_cv_lane_detection,
            debug_perspective=more_args.debug_perspective
        )

        record_video = more_args.record_video
        if record_video:
            video_recorder = BeamNGVideoRecorder(
                beamng_client,
                vehicle,
                beamng_steps_per_second=controller.beamng_steps_per_second,
                video_fps=float(controller.beamng_steps_per_second) / step_size,
                video_path=more_args.video_path,
                video_resolution=more_args.video_resolution,
                ffmpeg_path=more_args.ffmpeg_path,
            )

        blackboard.mark_running(run_id)
        beamng_client.settings.set_deterministic(controller.beamng_steps_per_second)
        beamng_client.pause()
        # If we step by less than 10, no camera image will be ready
        beamng_client.step(INITIAL_STEP_SIZE)

        i = 0
        while iterations < 0 or i < iterations:
            beamng_client.step(step_size)
            if record_video:
                video_recorder.record_frame()
                    
            controller.next_control(step_size)

            logger.debug("Command %02d/%d.", i + 1, iterations)

            if not blackboard.is_running(run_id):
                logger.info(
                    "Run %s is not running any more. Stopping controller...",
                    run_id,
                )
                break

            i += 1

    finally:
        if more_args.record_video:
            video_recorder.finalize()

        if vehicle is not None:
            stop_vehicle(vehicle)
        cv2.destroyAllWindows()
        beamng_client.disconnect()


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    set_up_simple_logging()
    logging.getLogger("visionpilot_controller").setLevel(logging.INFO)

    run_vehicle_controller(
        beamng_host=args.host,
        beamng_port=args.port,
        beamng_home=args.home,
        beamng_user=args.user,
        run_id=args.run_id,
        vehicle_id=args.vehicle_id,
        iterations=args.iterations,
        step_size=args.step_size,
        focus_vehicle=args.focus_vehicle,
        connect_timeout_s=args.connect_timeout_s,
        more_args=args,
    )


if __name__ == "__main__":
    main()
