"""PC-only keyboard direct drive acceptance: no FSM or obstacle checks.

The original standalone physics/fault lab stays available in the test suite,
but /keyboard uses only observed ROS2 Twist and direct body->world integration.
No real /odom, UART or motor control.
"""
import math
from pathlib import Path
import sys
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import keyboard_mapping_core as mapping
from keyboard_dryrun_chassis import KeyboardChassisTwin


def received(vx=0.,vy=0.,wz=0.):
    return {
        "source":"local_ros2",
        "topic":"/car/teleop/local_test_twist",
        "stamp_s":time.time(),
        "frame_id":"base_link",
        "vx":vx,"vy":vy,"wz":wz,
    }


def keys(seq,seq_keys,linear=.12,angular=.30):
    return {"action":"keys","client_seq":seq,"keys":list(seq_keys),
            "speed_mps":linear,"turn_rps":angular,"deadman":bool(seq_keys)}


class DirectKeyboardTests(unittest.TestCase):
    def test_default_local_preview_is_direct_not_physics(self):
        state=mapping.KeyboardMapping()
        row=state.digital_twin.snapshot()
        self.assertEqual(row["preview_mode"],"direct")
        self.assertFalse(row["real_odom"])
        self.assertFalse(row["hardware_authorized"])
        self.assertEqual(row["blocked_cells"],[])
        self.assertEqual(row["collision_reason"],"")

    def test_exact_ros2_twist_and_body_frame_motion(self):
        model=KeyboardChassisTwin(mode="direct")
        model.advance(0.,{"vx":.20,"vy":0.,"wz":0.},True)
        row=model.advance(.2,{"vx":.20,"vy":0.,"wz":0.},True)
        self.assertEqual(row["velocity"]["vx"],.2)  # NO acceleration
        self.assertAlmostEqual(row["x_m"],2.12,places=3)  # yaw=pi points west
        self.assertAlmostEqual(row["y_m"],2.16,places=3)
        self.assertEqual(row["model_error"]["linear_norm_mps"],0.)
        self.assertEqual(row["error_kind"],"DIRECT_ROS2_TWIST_TO_POSE")
        stopped=model.advance(.4,{"vx":0.,"vy":0.,"wz":0.},False)
        self.assertEqual(stopped["velocity"]["vx"],0.)
        self.assertEqual(stopped["state"],"STOPPED")

    def test_wasdqe_signs_and_rotation_without_task_steps(self):
        self.assertEqual(mapping.mapped_twist(["w"],.12,.30)["vx"],.12)
        self.assertEqual(mapping.mapped_twist(["s"],.12,.30)["vx"],-.12)
        self.assertEqual(mapping.mapped_twist(["a"],.12,.30)["vy"],.12)
        self.assertEqual(mapping.mapped_twist(["d"],.12,.30)["vy"],-.12)
        self.assertEqual(mapping.mapped_twist(["q"],.12,.30)["wz"],.30)
        self.assertEqual(mapping.mapped_twist(["e"],.12,.30)["wz"],-.30)
        model=KeyboardChassisTwin(mode="direct")
        model.advance(0,{"vx":0.,"vy":.1,"wz":.5},True)
        after=model.advance(.2,{"vx":0.,"vy":.1,"wz":.5},True)
        self.assertEqual(after["velocity"]["vy"],.1)
        self.assertEqual(after["velocity"]["wz"],.5)
        self.assertAlmostEqual(after["yaw_rad"],math.atan2(
            math.sin(math.pi+.1),math.cos(math.pi+.1)),places=4)

    def test_red_cell_and_fixed_station_never_block_direct_drive(self):
        model=KeyboardChassisTwin(mode="direct")
        self.assertEqual(model.reset("upper_right",[23])["blocked_cells"],[])
        for k in range(100):
            row=model.advance(k*.2,{"vx":.2,"vy":0.,"wz":0.},True)
        self.assertEqual(row["state"],"RUNNING")
        self.assertEqual(row["collision_reason"],"")
        self.assertLess(row["x_m"],0.)  # moves BEYOND virtual field; no clamp
        self.assertEqual(model.snapshot()["fault_mode"],"none")

    def test_no_reset_on_mode_switch_and_no_keyboard_without_ros2(self):
        state=mapping.KeyboardMapping()
        state.accept(keys(1,["w"]))
        before=state.dryrun_snapshot()
        self.assertEqual(before["input_twist"]["vx"],0.)
        state.observe_ros2(received(.12))
        state.dryrun_snapshot()
        state.digital_twin.last_time=time.monotonic()-.12
        moved=state.dryrun_snapshot()
        self.assertLess(moved["x_m"],before["x_m"])
        state.accept({"action":"route","client_seq":2,"route_mode":"pi_stm32"})
        self.assertEqual(state.digital_twin.snapshot()["x_m"],moved["x_m"])
        state.accept({"action":"route","client_seq":3,"route_mode":"local_ros2"})
        self.assertEqual(state.digital_twin.snapshot()["x_m"],moved["x_m"])
        self.assertFalse(state.effective()["active"])

    def test_local_mock_rejects_old_fault_fsm_without_real_motors(self):
        state=mapping.KeyboardMapping()
        with self.assertRaises(ValueError):
            state.dryrun_action({"action":"fault","fault_mode":"motor_stall"})
        self.assertFalse(state.digital_twin.snapshot()["hardware_authorized"])
        self.assertEqual(state.snapshot(
            {"online":False,"age_s":None,"sample":None}
        )["comparison"]["status"],"NO_REAL_ODOM")

    def test_browser_has_unarmed_keys_and_no_scene_auto_reset(self):
        ui=(ROOT/"web/keyboard_mapping_panel.js").read_text(encoding="utf-8")
        self.assertIn("routeMode==='local_ros2'?\n      active.size>0",ui)
        self.assertIn("routeMode!=='local_ros2'&&!enabled.checked",ui)
        self.assertNotIn("sim2real:field-changed",ui)
        self.assertNotIn("twinQa",ui)
        self.assertNotIn("twinFault",ui)
        self.assertIn("if(deadmanHeld()&&active.size>0&&!document.hidden)sendNow()",ui)


if __name__=="__main__":
    unittest.main(verbosity=2)
