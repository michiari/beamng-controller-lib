"""Command-line entry point using shared road sources and BeamNG lifecycle."""

if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from safe_mpd_controller.safe_mpd_controller import (
    build_parser as shared_parser, main as shared_main, load_controller_roads,
)
from safe_mpd_controller.utils import load_building_polygons

from dflat_controller.dflat import DFLATController
from safe_mpd_controller.beamng_roads import prepare_planning_roads


def build_parser():
    return shared_parser(DFLATController)


def build_controller(vehicle, args, *, beamng=None):
    if args.trailer:
        raise ValueError("DFLAT supports cars only; --trailer is not supported")
    roads = load_controller_roads(args, beamng)
    if args.beamng_roads:
        roads = prepare_planning_roads(
            roads, args.goal_pos, args.endpoint_tolerance_m, args.goal_runout_m,
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
        spawn_heading=getattr(args, "spawn_heading", None),
        road_height_offset_m=args.road_height_offset_m,
        script_cling=args.script_cling,
        # Dense float32 edge samples otherwise produce thousands of tiny walls.
        boundary_simplify_tolerance_m=1e-3 if args.beamng_roads else 0.0,
    )


def main():
    shared_main(DFLATController, build_controller)


if __name__ == "__main__":
    main()
