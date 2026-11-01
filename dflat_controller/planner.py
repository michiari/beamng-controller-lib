"""Run in DFLAT-AIT's Python environment, independently of BeamNG dependencies."""

import argparse
import json
from pathlib import Path
import sys
import tempfile


def plan(dflat_root, map_path, settings):
    sys.path.insert(0, str(dflat_root))
    import numpy as np
    import torch
    from run_dflat import build_parser, run_flat_diffusion, bench_metrics, report_and_save
    from lib import config, flat_lift
    from lib.device import DEVICE, seed_everything
    from lib.geometry.footprint import bind_footprint, bind_obstacles, vehicle_clearance
    from lib.world.maps import load_map, draw_fn
    from lib.world.robots import make_robot

    args = build_parser().parse_args([str(map_path)])
    vars(args).update(settings["planner"])
    robot = make_robot(args.model, **settings["robot"])
    bind_footprint(robot)
    flat_lift.bind_car(robot)
    mapd = load_map(map_path, device=DEVICE)
    bind_obstacles(mapd["polys"], bounds=mapd["bounds"], device=DEVICE)
    for label in ("start", "goal"):
        pose = torch.as_tensor(mapd[label][None], dtype=torch.float32, device=DEVICE)
        if float(vehicle_clearance(pose).item()) <= 0.0:
            raise RuntimeError(f"DFLAT {label} pose collides with the map or bounds")

    args.run_seed = config.resolve_seed(args.seed)
    seed_everything(args.run_seed)
    result = run_flat_diffusion(
        robot, mapd, mapd["start"], mapd["goal"],
        np.random.default_rng(args.run_seed), args,
    )
    if result is None:
        raise RuntimeError("DFLAT failed to find a trajectory")
    if settings.get("debug_output_dir") is not None:
        output_dir = Path(settings["debug_output_dir"])
        output_dir.mkdir(parents=True, exist_ok=True)
        artifacts_dir = Path(tempfile.mkdtemp(prefix="plan-", dir=output_dir))
        report_and_save(
            str(artifacts_dir / f"flat_diffusion_map_{args.model}"), robot,
            draw_fn(mapd), mapd["goal"], mapd["wall_pp"], result, panels=False,
        )
    metrics = bench_metrics(result, mapd["goal"], mapd["wall_pp"], mapd=mapd)
    if metrics["collision"] or metrics["ge"] > 1.0 or metrics["he"] > 15.0:
        raise RuntimeError(
            f"DFLAT trajectory rejected: clearance={metrics['min_clr']:.3f} m, "
            f"goal error={metrics['ge']:.3f} m, heading error={metrics['he']:.1f} deg"
        )
    return {"states": result["states"].tolist(), "dt": float(result["dt"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dflat_root", type=Path)
    parser.add_argument("map_path", type=Path)
    parser.add_argument("settings_path", type=Path)
    parser.add_argument("output_path", type=Path)
    args = parser.parse_args()
    settings = json.loads(args.settings_path.read_text(encoding="utf-8"))
    result = plan(args.dflat_root, args.map_path, settings)
    args.output_path.write_text(json.dumps(result, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
