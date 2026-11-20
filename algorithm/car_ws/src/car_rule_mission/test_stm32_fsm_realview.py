#!/usr/bin/env python3
"""End-to-end offline TEST lab tests: real anonymous PTY + MOCK STM32 feedback."""
import io
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "sim2real"))
import test_stm32_fsm_lab as lab
import web_mode_gateway


class HTTP:
    def __init__(self, path, payload=None, client="127.0.0.1", origin=None):
        self.path = path
        self.client_address = (client, 53100)
        body = json.dumps(payload).encode("utf-8") if payload is not None else b""
        self.rfile = io.BytesIO(body)
        self.headers = {"Content-Type": "application/json",
                        "Content-Length": str(len(body)),
                        "Host": "127.0.0.1:8090"}
        if origin:
            self.headers["Origin"] = origin
        self.response = None

    def respond_json(self, status, payload):
        self.response = (status, payload)


def start(**changes):
    data = {"action": "start", "start_zone": "upper_right",
            "task_code": "156+123+516+231", "blocked_roads": [],
            "fault": "none"}
    data.update(changes)
    return data


class MockSTM32RealViewTests(unittest.TestCase):
    def setUp(self):
        lab.reset_for_tests()

    def tearDown(self):
        lab.reset_for_tests()

    def test_reused_original_pty_simulator_executes_all_steps(self):
        response = lab.action(start())
        self.assertEqual(response["transport"], "ANONYMOUS_PTY_CARLINK_TEST_ONLY")
        self.assertEqual(response["model_kind"], "PYTHON_SIMEXECUTOR_NOT_COMPILED_STM32")
        self.assertFalse(response["hardware_authorized"])
        self.assertIsNotNone(lab.SESSION)
        self.assertGreaterEqual(lab.SESSION.master, 0)
        self.assertGreaterEqual(lab.SESSION.slave, 0)
        result = lab.action({"action": "run"})
        s = result["snapshot"]
        self.assertEqual(s["state"], "DONE", s["error"])
        self.assertEqual((s["pick_count"], s["place_count"], s["stack_count"]),
                         (12, 12, 3))
        self.assertTrue(s["home_confirmed"])
        self.assertEqual(s["completed_steps"], s["total_steps"])
        self.assertEqual(s["mock_executions"], s["total_steps"])
        self.assertEqual(s["pose_source"], "DISCRETE_ROAD_NODE")

    def test_one_step_and_subsequent_run(self):
        first = lab.action(start(start_zone="lower_right",
                                 blocked_roads=["B3-C3"]))
        self.assertEqual(first["snapshot"]["state"], "READY")
        second = lab.action({"action": "step"})["snapshot"]
        self.assertEqual(second["completed_steps"], 1)
        self.assertEqual(second["state"], "READY")
        third = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(third["state"], "DONE")

    def test_actuator_failure_latches_and_blocks_future_step(self):
        lab.action(start(fault="failed_at_3"))
        error = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(error["state"], "FAULT")
        self.assertEqual(error["mock_executions"], 3)
        self.assertEqual(error["completed_steps"], 2)
        repeat = lab.action({"action": "step"})
        again = repeat["snapshot"]
        self.assertEqual(again["mock_executions"], 3)
        self.assertIsNone(repeat["transition"])

    def test_ack_dropout_latches_and_does_not_repeat_command(self):
        lab.action(start(fault="missing_ack_at_2"))
        failed = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(failed["state"], "FAULT")
        self.assertIn("TIMEOUT", failed["error"])
        self.assertEqual(failed["mock_executions"], 1)

    def test_bad_start_does_not_reset_existing_pty(self):
        lab.action(start())
        original = lab.SESSION
        for payload in (start(task_code="156+112+516+231"),
                        start(start_zone="invalid"),
                        start(blocked_roads=["A1-A2", "A1-B1", "A2-B2", "A2-A3",
                                             "B1-B2", "B2-B3", "C1-C2", "C2-C3", "X9-Z9"]),
                        start(fault="real_uart")):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                lab.action(payload)
            self.assertIs(lab.SESSION, original)

    def test_http_test_only_not_real_and_localhost_post(self):
        p = lab.LAB_PATH
        self.assertFalse(lab.handle_get(HTTP("/api/real/stm32_fsm_lab")))
        self.assertFalse(lab.handle_post(HTTP("/api/real/stm32_fsm_lab", start())))
        for client, origin in (("10.8.0.3", None),
                               ("127.0.0.1", "http://evil.invalid")):
            with self.subTest(client=client,origin=origin):
                req = HTTP(p, start(), client=client, origin=origin)
                self.assertTrue(lab.handle_post(req))
                self.assertEqual(req.response[0], 403)
        req = HTTP(p, start())
        self.assertTrue(lab.handle_post(req))
        self.assertEqual(req.response[0], 200)
        self.assertFalse(req.response[1]["hardware_authorized"])
        get = HTTP(p)
        self.assertTrue(lab.handle_get(get))
        self.assertEqual(get.response[0], 200)

    def test_reject_fake_real_and_command_injection(self):
        lab.action(start())
        for payload in ({"action": "move"}, {"action": "step", "cmd_vel": 1},
                        {"action": "start", "source_type": "REAL"},
                        {"action": "reset", "hardware_authorized": True}):
            req = HTTP(lab.LAB_PATH, payload)
            self.assertTrue(lab.handle_post(req))
            self.assertEqual(req.response[0], 400)
        self.assertEqual(lab.status()["snapshot"]["state"], "READY")

    def test_separate_real_channel_is_not_mutated(self):
        before = web_mode_gateway.CHANNELS["real"]["ros"]["seq"]
        lab.action(start())
        lab.action({"action": "run"})
        after = web_mode_gateway.CHANNELS["real"]["ros"]["seq"]
        self.assertEqual(before, after)
        self.assertIsNone(web_mode_gateway.CHANNELS["real"]["ros"]["snapshot"])
        self.assertFalse(lab.status()["hardware_authorized"])

    def test_native_c_mode_scans_before_running_34_material_actions(self):
        import shutil
        if shutil.which("gcc") is None:
            self.skipTest("Host GCC required for real native C mode")
        response = lab.action(start(engine="code_fsm"))
        self.assertEqual(response["transport"], "NATIVE_GCC_C_FSM_NO_UART")
        self.assertFalse(response["hardware_authorized"])
        self.assertEqual(response["snapshot"]["total_steps"], 42)
        self.assertEqual(response["snapshot"]["phase"], "QR_ROUTE")
        self.assertEqual(response["snapshot"]["node"], "C3")
        steps = [lab.action({"action": "step"}) for _ in range(8)]
        self.assertEqual(steps[0]["snapshot"]["kind"], "QR_MOVE")
        self.assertEqual(steps[1]["transition"]["to_node"], "C2")
        self.assertEqual(steps[4]["snapshot"]["kind"], "SCAN_QR")
        self.assertEqual(steps[4]["snapshot"]["event"], "QR_CONFIRMED")
        self.assertGreater(steps[4]["snapshot"]["qr_session"], 0)
        self.assertTrue(steps[7]["snapshot"]["qr_valid"])
        self.assertEqual(steps[7]["snapshot"]["phase"], "TASK")
        first_pick = lab.action({"action": "step"})
        self.assertEqual(first_pick["snapshot"]["kind"], "PICK")
        self.assertEqual(first_pick["snapshot"]["completed_steps"], 9)
        final = lab.action({"action": "run"})["snapshot"]
        self.assertEqual((final["state"], final["completed_steps"]), ("DONE", 42))
        self.assertEqual((final["pick_count"], final["place_count"],
                          final["stack_count"]), (12, 15, 3))
        self.assertTrue(final["home_confirmed"])

    def test_native_c_fault_latches_without_phantom_map_motion(self):
        import shutil
        if shutil.which("gcc") is None:
            self.skipTest("Host GCC required")
        lab.action(start(engine="code_fsm", fault="missing_ack_at_2"))
        stopped = lab.action({"action": "run"})
        self.assertEqual(stopped["snapshot"]["state"], "FAULT")
        self.assertEqual(stopped["snapshot"]["completed_steps"], 1)
        self.assertEqual(stopped["snapshot"]["error"], "INJECTED_LINK_FAILURE")
        self.assertFalse(stopped["transition"]["executed"])
        self.assertEqual(len(stopped["transition"]["points"]), 1)
        again = lab.action({"action": "step"})
        self.assertEqual(again["snapshot"]["completed_steps"], 1)

    def test_c_qr_timeout_never_builds_material_task(self):
        import shutil
        if shutil.which("gcc") is None:
            self.skipTest("Host GCC required")
        lab.action(start(engine="code_fsm", fault="qr_timeout"))
        result = lab.action({"action": "run"})
        self.assertEqual(result["snapshot"]["state"], "FAULT")
        self.assertEqual(result["snapshot"]["error"], "SCAN_TIMEOUT")
        self.assertEqual(result["snapshot"]["phase"], "QR_ROUTE")
        self.assertEqual(result["snapshot"]["pick_count"], 0)
        self.assertEqual(result["snapshot"]["completed_steps"], 4)
        self.assertFalse(result["snapshot"]["qr_valid"])

    def test_red_cell_17_blocks_b2_b3_in_both_engines(self):
        from field_road_mapping import blocked_roads_from_cells
        derived, mapping = blocked_roads_from_cells([17])
        self.assertIn("B2-B3", derived)
        self.assertEqual(mapping[0]["cell"], 17)
        self.assertNotIn("A2-B2", derived)
        preview = lab.action({"action": "preview",
                              "blocked_cells": [17], "blocked_roads": []})
        self.assertIn("B2-B3", preview["preview"]["blocked_roads"])
        for engine in ("pty", "code_fsm"):
            with self.subTest(engine=engine):
                response = lab.action(start(engine=engine, blocked_cells=[17]))
                self.assertIn("B2-B3", response["case"]["blocked_roads"])
                self.assertEqual(response["case"]["blocked_cells"], [17])
                if engine == "pty":
                    lab.action({"action": "run"})
                    self.assertEqual(lab.status()["snapshot"]["state"], "DONE")
                else:
                    # The C QR approach route must detour around the blocked
                    # B2-B3 edge before reaching material B3.
                    for _ in range(7):
                        step_result = lab.action({"action": "step"})
                        path = step_result["transition"]["node_path"]
                        self.assertFalse(any({a, b} == {"B2", "B3"}
                                             for a, b in zip(path, path[1:])))
                    self.assertEqual(lab.status()["snapshot"]["node"], "B3")

    def test_invalid_cell_ids_rejected(self):
        for cells in ([25], [-1], [True], [17, 17], [0, 1, 2], "17"):
            with self.subTest(cells=cells), self.assertRaises(ValueError):
                lab.action(start(blocked_cells=cells))

    def test_pty_road_step_has_visualizable_source_and_destination(self):
        lab.action(start())
        first = lab.action({"action": "step"})
        transition = first["transition"]
        self.assertEqual(first["engine"], "pty")
        self.assertEqual(transition["kind"], "MOVE_NODE")
        self.assertTrue(transition["executed"])
        self.assertNotEqual(transition["from_node"], transition["to_node"])
        self.assertEqual(len(transition["points"]), 2)
        for point in transition["points"]:
            self.assertEqual(len(point), 2)
            self.assertTrue(all(0 <= v <= 2.4 for v in point))
        # Virtual road-node preview is never real odometry.
        self.assertEqual(first["snapshot"]["pose_source"], "DISCRETE_ROAD_NODE")

    def test_switching_engines_closes_previous_anonymous_pty(self):
        import shutil
        if shutil.which("gcc") is None:
            self.skipTest("Host GCC required")
        lab.action(start(engine="pty"))
        original = lab.SESSION
        lab.action(start(engine="code_fsm"))
        self.assertEqual((original.master, original.slave), (-1, -1))
        self.assertEqual(lab.status()["snapshot"]["total_steps"], 42)
        lab.action(start(engine="pty"))
        self.assertEqual(lab.status()["snapshot"]["total_steps"],
                         len(lab.SESSION.steps))
        self.assertEqual(lab.status()["transport"], "ANONYMOUS_PTY_CARLINK_TEST_ONLY")

    def test_web_assets_are_test_only_and_first_screen_map(self):
        web = ROOT / "sim2real" / "web"
        body = (web / "index.html").read_text(encoding="utf-8")
        bench = (web / "test_stm32_fsm_workbench.js").read_text(encoding="utf-8")
        layout = (web / "field_ui_hierarchy.js").read_text(encoding="utf-8")
        style = (web / "field_hierarchy_v2.css").read_text(encoding="utf-8")
        self.assertIn("test_stm32_fsm_workbench.js", body)
        self.assertNotIn("real_mock_stm32_fsm.js", body)
        self.assertIn("mode !== 'test'", bench)
        self.assertIn("nav.after(map)", layout)
        self.assertIn(".sidebar .settings-directory", style)
        self.assertIn("C 代码状态机", bench)
        editor = (web / "app.js").read_text(encoding="utf-8")
        self.assertIn("getTestFieldSnapshot", editor)
        self.assertIn("sim2real:field-changed", editor)
        self.assertIn("blocked_cells:fieldSnapshot?.blocked_cells||[]", bench)
        self.assertIn("SCAN_QR", (ROOT / "sim2real" /
                                  "test_stm32_fsm_lab.py").read_text(encoding="utf-8"))

    def test_map_cell17_blocks_middle_vertical_lane_and_detours(self):
        from field_road_mapping import blocked_roads_from_cells
        blocked, mapping = blocked_roads_from_cells([17])
        self.assertIn("B2-B3", blocked)
        self.assertNotIn("A2-B2", blocked)
        self.assertEqual(mapping[0]["cell"], 17)
        response = lab.action(start(engine="code_fsm", blocked_cells=[17]))
        self.assertIn("B2-B3", response["case"]["blocked_roads"])
        self.assertEqual(response["case"]["mapped_roads"], blocked)
        # Native QR prelude includes a route from C2 to material B3.
        for _ in range(6):
            step = lab.action({"action": "step"})
        nodes = step["transition"]["node_path"]
        self.assertNotIn(("B2", "B3"), list(zip(nodes, nodes[1:])))
        self.assertNotIn(("B3", "B2"), list(zip(nodes, nodes[1:])))

    def test_native_scan_timeout_cannot_start_task_plan(self):
        import shutil
        if shutil.which("gcc") is None:
            self.skipTest("Host GCC required")
        lab.action(start(engine="code_fsm", fault="qr_timeout"))
        snap = lab.action({"action": "run"})["snapshot"]
        self.assertEqual(snap["state"], "FAULT")
        self.assertEqual(snap["completed_steps"], 4)
        self.assertFalse(snap["qr_valid"])
        self.assertEqual(snap["phase"], "QR_ROUTE")
        self.assertEqual(snap["pick_count"], 0)

    def test_stop_reset_invalidates_previous_step_generation(self):
        initialized = lab.action(start())
        old_generation = initialized["generation"]
        first = lab.action({"action": "step",
                            "expected_generation": old_generation})
        self.assertEqual(first["snapshot"]["completed_steps"], 1)
        cleared = lab.action({"action": "reset"})
        self.assertGreater(cleared["generation"], old_generation)
        self.assertIsNone(cleared["snapshot"])
        restarted = lab.action(start())
        self.assertEqual(restarted["snapshot"]["completed_steps"], 0)
        with self.assertRaisesRegex(ValueError, "STALE_MOCK_SESSION"):
            lab.action({"action": "step",
                        "expected_generation": old_generation})
        self.assertEqual(lab.status()["snapshot"]["completed_steps"], 0)
        step = lab.action({"action": "step",
                           "expected_generation": restarted["generation"]})
        self.assertEqual(step["snapshot"]["completed_steps"], 1)

    def test_stop_reset_rejects_delayed_old_start(self):
        first = lab.action(start())
        before = first["generation"]
        lab.action({"action": "reset"})
        stale = start()
        stale["expected_generation"] = before
        with self.assertRaisesRegex(ValueError, "STALE_MOCK_SESSION"):
            lab.action(stale)
        self.assertIsNone(lab.status()["snapshot"])

    def test_native_c_motion_warns_about_direct_unplanned_targets(self):
        from pathlib import Path
        c = (ROOT / "STM32F4" / "App" / "race_fsm.c").read_text(
            encoding="utf-8")
        board = (ROOT / "STM32F4" / "App" / "app_race.c").read_text(
            encoding="utf-8")
        self.assertIn("Race_StartQrRoute", c)
        self.assertIn("RACE_ACTION_MOVE", c)
        self.assertIn("Mecanum_SetPositionMode", board)
        self.assertNotIn("plan_mission", c)
        self.assertNotIn("blocked_roads", c)
        init = lab.action(start(engine="code_fsm"))
        self.assertFalse(init["native_c_motion_obstacle_aware"])
        self.assertEqual(init["native_c_motion_control"],
                         "DIRECT_POSITION_GOALS_NO_BLOCKED_ROAD_INPUT")
        self.assertEqual(init["motion_provenance"],
                         "PYTHON_DIJKSTRA_VISUAL_PREVIEW_NOT_NATIVE_C_MOTION")
        web = (ROOT / "sim2real" / "web" /
               "test_stm32_fsm_workbench.js").read_text(encoding="utf-8")
        self.assertIn("C 固件的扫码移动使用固定坐标目标", web)

    def test_reset_closes_pty_descriptors(self):
        lab.action(start())
        old = lab.SESSION
        lab.action({"action": "reset"})
        self.assertEqual((old.master, old.slave), (-1, -1))
        self.assertIsNone(lab.status()["snapshot"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
