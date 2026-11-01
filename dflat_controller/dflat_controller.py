"""Command-line entry point using the shared OpenDRIVE/BeamNG lifecycle."""

from beamngpy import Scenario

from safe_mpd_controller.safe_mpd_controller import build_parser as shared_parser, main as shared_main
from safe_mpd_controller.utils import load_building_polygons
from safe_mpd_controller.xodr_import import OpenDriveExtendedImporter

from .dflat import DFLATController


def build_parser():
    return shared_parser(DFLATController)


def build_controller(vehicle, args):
    if args.trailer:
        raise ValueError("DFLAT-AIT supports cars only; --trailer is not supported")
    roads = OpenDriveExtendedImporter.import_xodr(
        str(args.xodr_file), Scenario("tech_ground", "dflat_xodr_import"),
        {
            "material": "road_asphalt_light", "default_width": args.lane_width * 2.0,
            "drivability": 1, "texture_length": 50.0,
            "surface_type": "decal", "min_elevation": 0.0,
        },
    )
    return DFLATController(
        vehicle, roads, args.lane_width, args.spawn_pos, args.goal_pos,
        dflat_root=args.dflat_root, dflat_python=args.dflat_python,
        model=args.model, seed=args.seed, samples=args.samples,
        diffusion_steps=args.diffusion_steps, planning_timeout_s=args.planning_timeout_s,
        time_step=args.time_step, max_speed=args.max_speed, wheelbase=args.wheelbase,
        front_overhang=args.front_overhang, rear_overhang=args.rear_overhang,
        vehicle_width=args.vehicle_width, steering_limit_deg=args.steering_limit_deg,
        rear_axle_offset_m=args.rear_axle_offset_m,
        buildings=load_building_polygons(args.buildings) if args.buildings else None,
        debug=args.debug,
        debug_output_dir=args.debug_output_dir,
    )


def main():
    shared_main(DFLATController, build_controller)


if __name__ == "__main__":
    main()
