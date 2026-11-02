import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from dflat_controller import DFLATController
from dflat_controller.dflat_controller import build_parser


ROADS = [{"rid": "straight", "nodes": [(0, 0, 0, 10), (30, 0, 0, 10)]}]


class DFLATControllerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / "run_dflat.py").touch()

    def controller(self, **kwargs):
        return DFLATController(
            None, ROADS, 5.0, (5, 0, 2), (20, 0, 2),
            dflat_root=self.root, **kwargs,
        )

    def write_result(self, command, **kwargs):
        self.exported_map = json.loads(Path(command[-3]).read_text())
        self.settings = json.loads(Path(command[-2]).read_text())
        Path(command[-1]).write_text(json.dumps({
            "states": [[4, 0, 0], [10, 0, math.pi / 2], [19, 0, 0]], "dt": 0.5,
        }))

    def test_plans_with_isolated_python_and_converts_rear_axle_to_script(self):
        controller = self.controller(rear_axle_offset_m=1.0, dflat_python="planner-python")
        with patch("dflat_controller.dflat.subprocess.run", side_effect=self.write_result) as run:
            script = controller.plan_entire_route()
        self.assertEqual(run.call_args.args[0][0], "planner-python")
        self.assertEqual(run.call_args.kwargs["timeout"], 300.0)
        self.assertEqual(self.exported_map["start"], [4, 0, 0])
        self.assertEqual(self.exported_map["goal"], [19, 0, 0])
        self.assertTrue(self.exported_map["polys"])
        self.assertEqual(self.settings["robot"]["l"], 2.7)
        self.assertEqual(self.settings["planner"]["model"], "car")
        self.assertFalse(self.settings["planner"]["denoise_gif"])
        self.assertIsNone(self.settings["debug_output_dir"])
        self.assertEqual([node["t"] for node in script], [0.0, 0.5, 1.0])
        self.assertEqual(script[0], {"x": 5.0, "y": 0.0, "z": 2.0, "t": 0.0})
        self.assertAlmostEqual(script[1]["x"], 10.0)
        self.assertAlmostEqual(script[1]["y"], 1.0)
        self.assertEqual(script[-1]["x"], 20.0)
        self.assertEqual(controller.get_true_spawn_pos(), (5, 0, 0))
        self.assertEqual(controller.get_true_goal_pos(), (20, 0, 0))
        np.testing.assert_allclose(controller.trajectory_states[:, 0], [4, 10, 19])
        vehicle = Mock()
        controller.set_vehicle(vehicle)
        controller.control_entire_route()
        vehicle.ai.set_script.assert_called_once_with(script, cling=True)
        self.assertFalse(Path(run.call_args.args[0][-1]).exists())

    def test_debug_output_path_is_resolved_and_passed_to_worker(self):
        controller = self.controller(debug_output_dir="planning-debug")
        with patch("dflat_controller.dflat.subprocess.run", side_effect=self.write_result):
            controller.plan_entire_route()
        self.assertEqual(self.settings["debug_output_dir"], str(Path("planning-debug").resolve()))
        self.assertTrue(self.settings["planner"]["denoise_gif"])

    def test_none_lane_width_preserves_imported_widths(self):
        roads = [{"rid": "variable", "nodes": [(0, 0, 0, 4), (30, 0, 0, 8)]}]
        controller = DFLATController(
            None, roads, None, (5, 0, 0), (20, 0, 0), dflat_root=self.root,
        )
        self.assertEqual([node[3] for node in controller.builder.xodr_roads[0].nodes], [4.0, 8.0])

    def test_script_uses_road_elevation_and_configured_clinging(self):
        roads = [{"rid": "slope", "nodes": [(0, 0, 2, 10), (30, 0, 8, 10)]}]
        controller = DFLATController(
            None, roads, None, (5, 0, 3), (20, 0, 6), dflat_root=self.root,
            road_height_offset_m=0.25, script_cling=False,
        )
        with patch("dflat_controller.dflat.subprocess.run", side_effect=self.write_result):
            script = controller.plan_entire_route()
        np.testing.assert_allclose([point["z"] for point in script], [3.05, 4.25, 6.05])
        vehicle = Mock()
        controller.set_vehicle(vehicle)
        controller.control_entire_route()
        vehicle.ai.set_script.assert_called_once_with(script, cling=False)

    def test_failed_replan_clears_previous_script_and_propagates_error(self):
        controller = self.controller()
        controller.ai_script = [{"x": 1}]
        with patch("dflat_controller.dflat.subprocess.run", side_effect=subprocess.TimeoutExpired("planner", 1)):
            with self.assertRaises(subprocess.TimeoutExpired):
                controller.plan_entire_route()
        self.assertIsNone(controller.ai_script)
        self.assertIsNone(controller.trajectory_states)
        with self.assertRaisesRegex(ValueError, "Vehicle must be set"):
            controller.control_entire_route()

    def test_rejects_malformed_worker_trajectories(self):
        for result in (
            {"states": [[1, 2, 3]], "dt": 0.25},
            {"states": [[1, 2]] * 3, "dt": 0.25},
            {"states": [[1, 2, math.nan]] * 3, "dt": 0.25},
            {"states": [[1, 2, 3]] * 3, "dt": 0},
        ):
            with self.subTest(result=result):
                controller = self.controller()
                def write(command, **kwargs):
                    Path(command[-1]).write_text(json.dumps(result))
                with patch("dflat_controller.dflat.subprocess.run", side_effect=write):
                    with self.assertRaises(ValueError):
                        controller.plan_entire_route()
                self.assertIsNone(controller.ai_script)

    def test_rejects_invalid_configuration(self):
        for settings in (
            {"trailer": True}, {"model": "truck"}, {"samples": 0},
            {"diffusion_steps": 1.5}, {"max_speed": math.nan},
            {"planning_timeout_s": -1}, {"steering_limit_deg": 90},
            {"wheelbase": 0}, {"rear_axle_offset_m": math.inf},
            {"spawn_heading": math.nan},
            {"road_height_offset_m": math.nan},
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                self.controller(**settings)
        (self.root / "run_dflat.py").unlink()
        with self.assertRaisesRegex(ValueError, "run_dflat.py not found"):
            self.controller()

    def test_cli_combines_connection_and_dflat_arguments(self):
        args = build_parser().parse_args([
            "--dflat-root", str(self.root), "--xodr-file", "road.xodr",
            "--lane-width", "5", "--spawn-pos", "5", "0", "2",
            "--goal-pos", "20", "0", "2", "--model", "acc",
            "--vehicle-id", "ego", "--rear-axle-offset-m", "1.2",
            "--debug-output-dir", "planning-debug",
        ])
        self.assertEqual(args.model, "acc")
        self.assertEqual(args.vehicle_id, "ego")
        self.assertEqual(args.rear_axle_offset_m, 1.2)
        self.assertEqual(args.spawn_pos, [5, 0, 2])
        self.assertEqual(args.debug_output_dir, Path("planning-debug"))

    @unittest.skipUnless(os.environ.get("DFLAT_AIT_ROOT"), "Set DFLAT_AIT_ROOT for real planner test")
    def test_real_planner_saves_png_gif_and_csv(self):
        output_dir = self.root / "debug"
        controller = DFLATController(
            None, ROADS, 5.0, (5, 0, 0), (10, 0, 0),
            dflat_root=os.environ["DFLAT_AIT_ROOT"],
            dflat_python=os.environ.get("DFLAT_PYTHON"),
            samples=8, diffusion_steps=2, planning_timeout_s=120,
            debug_output_dir=output_dir,
        )
        controller.plan_entire_route()
        directories = list(output_dir.glob("plan-*"))
        self.assertEqual(len(directories), 1)
        stem = directories[0] / "flat_diffusion_map_car"
        self.assertTrue(stem.with_suffix(".png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
        self.assertIn(stem.with_suffix(".gif").read_bytes()[:6], (b"GIF87a", b"GIF89a"))
        self.assertTrue(stem.with_suffix(".csv").read_text().startswith("t,x,y,theta"))
        self.assertIsNotNone(controller.ai_script)

    @unittest.skipUnless(os.environ.get("DFLAT_AIT_ROOT"), "Set DFLAT_AIT_ROOT for real planner test")
    def test_real_planner_on_straight_road(self):
        for model in ("car", "acc"):
            with self.subTest(model=model):
                controller = DFLATController(
                    None, ROADS, 5.0, (5, 0, 0), (20, 0, 0),
                    dflat_root=os.environ["DFLAT_AIT_ROOT"],
                    dflat_python=os.environ.get("DFLAT_PYTHON"),
                    model=model, samples=8, diffusion_steps=2, planning_timeout_s=60,
                )
                script = controller.plan_entire_route()
                self.assertGreaterEqual(len(script), 3)
                self.assertLess(math.hypot(script[-1]["x"] - 20, script[-1]["y"]), 1.0)
                self.assertTrue(all(b["t"] > a["t"] for a, b in zip(script, script[1:])))


if __name__ == "__main__":
    unittest.main()
