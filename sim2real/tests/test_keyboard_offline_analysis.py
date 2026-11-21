"""No ROS2/STM32 needed: transport sequence, replay, step and mapping tests."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# Grouped modules remain importable under direct unittest discovery.
import pathlib as _sim2real_pathlib
for _module_group in ("keyboard", "bridges", "labs"):
    sys.path.insert(0, str(_sim2real_pathlib.Path(__file__).resolve().parents[1] / _module_group))
import keyboard_offline_analysis as a
from keyboard_ros2_pi import CandidateGuard


def packet(seq, sent=10., received=10.02, channel="pc_local_ros2"):
    return {"kind": "packet", "channel": channel, "session": "test",
            "seq": seq, "sent_s": sent, "received_s": received}


def step(t, desired, actual, origin="MOCK_KINEMATIC_ONLY"):
    return {"kind": "step", "t_s": t, "desired_mps": desired,
            "response_mps": actual, "source_type": origin}


class OfflineTransportTests(unittest.TestCase):
    def test_packet_drops_reordered_and_duplicate_calculated_from_unique_span(self):
        samples = [packet(10, 1, 1.01),
                   packet(11, 1.05, 1.06),
                   packet(14, 1.2, 1.21),
                   packet(11, 1.05, 1.22),
                   packet(13, 1.15, 1.23),
                   packet(15, 1.25, 1.26)]
        item = a.delivery_metrics(samples)[0]
        self.assertEqual(item["unique"], 5)
        self.assertEqual(item["missing_in_observed_range"], 1)
        self.assertEqual(item["duplicates"], 1)
        self.assertEqual(item["reordered"], 1)
        self.assertFalse(item["source_type"].startswith("REAL"))
        self.assertGreaterEqual(item["receive_jitter_rms_ms"], 0)

    def test_packet_timestamp_and_id_errors_fail(self):
        good = packet(0)
        for row in ({**good, "channel": "REAL_ODOM"},
                    {**good, "seq": -1},
                    {**good, "seq": True},
                    {**good, "session": ""},
                    {**good, "sent_s": float("nan")},
                    {**good, "received_s": good["sent_s"]-.5},
                    {**good, "motor_authorized": True}):
            with self.subTest(row=row), self.assertRaises(ValueError):
                a.valid_packet(row)

    def test_separate_pi_and_local_sessions(self):
        series = [packet(1), packet(2), packet(20, channel="pi_candidate")]
        rows = a.delivery_metrics(series)
        self.assertEqual(len(rows), 2)
        self.assertEqual([r["missing_in_observed_range"] for r in rows], [0, 0])

    def test_finite_percentile_and_empty(self):
        self.assertIsNone(a.percentile([], .95))
        self.assertEqual(a.percentile([1, 5, 10], .5), 5)
        self.assertEqual(a.percentile([1, 5, 10], .95), 9.5)

    def test_replay_jsonl_roundtrip_reject_unknown_or_nonfinite(self):
        events = a.generate_fixture()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "session.jsonl"
            import json
            path.write_text("".join(json.dumps(x)+"\n" for x in events),
                            encoding="utf-8")
            replayed = a.read_jsonl(path)
            self.assertEqual(len(replayed), len(events))
            first = a.replay(events)
            second = a.replay(replayed)
            self.assertEqual(first, second)
            self.assertFalse(first["stm32_tx_enabled"])
            self.assertEqual(first["step_response"]["source_type"],
                             "MOCK_KINEMATIC_ONLY")
            path.write_text('{"kind":"packet", "stamp":NaN}\n',
                            encoding="utf-8")
            with self.assertRaises(ValueError):
                a.read_jsonl(path)
            path.write_text('{"kind":"unsafe_uart","cmd_vel":1}\n',
                            encoding="utf-8")
            with self.assertRaises(ValueError):
                a.read_jsonl(path)

    def test_generated_fixture_loss_and_step_are_deterministic(self):
        left = a.replay(a.generate_fixture())
        right = a.replay(a.generate_fixture())
        self.assertEqual(left, right)
        s = left["packets"][0]
        self.assertEqual(s["missing_in_observed_range"], 3)
        self.assertEqual(s["duplicates"], 1)
        self.assertEqual(s["seq_range"], [0, 79])
        self.assertIsNotNone(left["step_response"]["rise_time_10_90_s"])
        self.assertIsNotNone(left["step_response"]["settling_time_5pct_s"])

    def test_mecanum_roundtrip_and_signs_are_assumptions_not_wiring(self):
        for value in ((.15, 0., 0.), (0., .10, 0.),
                      (0., 0., .3), (.06, -.04, -.15)):
            wheels = a.mecanum_wheels(*value)
            returned = a.mecanum_body(wheels)
            for expected, actual in zip(value, returned):
                self.assertAlmostEqual(expected, actual)
        left = a.mecanum_wheels(0, .1, 0)
        self.assertLess(left[0], 0)
        self.assertGreater(left[1], 0)
        with self.assertRaises(ValueError):
            a.mecanum_wheels(1, 0, 0, radius=0)
        with self.assertRaises(ValueError):
            a.mecanum_body([0, 0, float("nan"), 0])

    def test_body_frame_to_world_trace_and_heading(self):
        trace = a.reference_trace([
            {"t_s": 0.0, "vx": .1, "vy": 0., "wz": 0.},
            {"t_s": 0.2, "vx": .1, "vy": 0., "wz": 0.}],
            initial=(.5, .5, math.pi/2))
        self.assertAlmostEqual(trace[-1][0], .5, places=3)
        self.assertAlmostEqual(trace[-1][1], .52, places=3)
        with self.assertRaises(ValueError):
            a.reference_trace([
                {"t_s": 0., "vx": .1, "vy": 0., "wz": 0.},
                {"t_s": 1.0, "vx": .1, "vy": 0., "wz": 0.}])

    def test_step_metrics_cannot_mix_mock_real_or_nonstep(self):
        rows = [step(0, 0, 0), step(.1, .1, .01),
                step(.2, .1, .08), step(.3, .1, .099),
                step(.4, .1, .10)]
        result = a.step_metrics(rows)
        self.assertAlmostEqual(result["step_t_s"], .1)
        self.assertAlmostEqual(result["target_command_mps"], .1)
        self.assertEqual(result["source_type"], "MOCK_KINEMATIC_ONLY")
        with self.assertRaises(ValueError):
            a.step_metrics(rows[:-1]+[step(.4, .1, .10, "REAL_ODOM_RECORDED_OFFLINE")])
        with self.assertRaises(ValueError):
            a.step_metrics([step(0, 0, 0)]*4)
        with self.assertRaises(ValueError):
            a.step_metrics([step(.1, 0, 0), step(.1, .1, .1),
                            step(.2, .1, .1), step(.3, .1, .1)])


class PiGuardOfflineTests(unittest.TestCase):
    def test_valid_rx_then_watchdog_timeout(self):
        gate = CandidateGuard()
        self.assertTrue(gate.receive(.15, .0, .2, "base_link",
                                     100.0, 100.02, 10.0))
        row = gate.tick(10.04)
        self.assertEqual(row["state"], "ROS2_CANDIDATE_ONLY")
        self.assertEqual(row["candidate"], (.15, .0, .2))
        self.assertFalse(row["uart_tx_enabled"])
        timed = gate.tick(10.301)
        self.assertEqual(timed["state"], "WATCHDOG_STOP")
        self.assertEqual(timed["candidate"], (0., 0., 0.))
        self.assertEqual(timed["reason"], "WATCHDOG_EXPIRED")

    def test_bad_frame_clock_replay_and_speed_fail_closed(self):
        for item, reason in [
                ((.1, 0, 0, "odom", 100, 100, 10), "INVALID_VELOCITY_OR_FRAME"),
                ((1.5, 0, 0, "base_link", 100, 100, 10), "INVALID_VELOCITY_OR_FRAME"),
                ((.1, 0, 0, "base_link", 100, 100.5, 10), "STALE_OR_FUTURE_STAMP"),
                ((.1, 0, 0, "base_link", 101, 100.5, 10), "STALE_OR_FUTURE_STAMP"),
                ((.1, 0, 0, "base_link", float("nan"), 100, 10), "INVALID_TIME")]:
            gate = CandidateGuard()
            self.assertFalse(gate.receive(*item))
            self.assertEqual(gate.tick(10)["candidate"], (0., 0., 0.))
            self.assertEqual(gate.reason, reason)
        gate = CandidateGuard()
        self.assertTrue(gate.receive(.1, 0, 0, "base_link", 100, 100, 10))
        self.assertFalse(gate.receive(.1, 0, 0, "base_link", 100, 100, 10.1))
        self.assertEqual(gate.reason, "DUPLICATE_OR_REORDERED_STAMP")
        self.assertEqual(gate.tick(10.1)["candidate"], (0., 0., 0.))

    def test_guard_rejects_time_reverse_even_after_valid(self):
        gate = CandidateGuard()
        gate.receive(.1, 0, 0, "base_link", 100, 100, 10)
        report = gate.tick(9.5)
        self.assertEqual(report["state"], "WATCHDOG_STOP")
        self.assertFalse(report["motor_authorized"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
