"""Run with CPython; only tests protocol math and source properties, not camera sensor."""
import ast
import importlib.util
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OPENMV = ROOT / 'src_openmv'
ROS = ROOT / 'ros2'

# Extract pure ROS2 protocol class to avoid importing ROS dependencies on CI.
code = ast.parse((ROS / 'serial_bridge_node.py').read_text())
proto_def = next(n for n in code.body if isinstance(n, ast.ClassDef) and n.name == 'Protocol')
proto_ns = {}
exec(compile(ast.Module(body=[proto_def], type_ignores=[]), '<Protocol>', 'exec'), proto_ns)
Protocol = proto_ns['Protocol']

# MicroPython stubs let ordinary CPython validate math and framing.
sys.path.insert(0, str(OPENMV))
image_mod = types.ModuleType('image')
display_mod = types.ModuleType('display')
machine_mod = types.ModuleType('machine')

class FakeUART:
    data = bytearray()
    def __init__(self, *args, **kw):
        self.tx = bytearray()
        self.rx = bytearray()
    def any(self):
        return len(self.rx)
    def read(self, n):
        data = bytes(self.rx[:n]); del self.rx[:n]; return data
    def write(self, s):
        self.tx.extend(s)

machine_mod.UART = FakeUART
sys.modules.update(image=image_mod, display=display_mod, machine=machine_mod)
import config as C
import utils as U


class WireTests(unittest.TestCase):
    def test_python_to_openmv(self):
        pc = Protocol()
        mv = U.PacketSerial()
        pc_message = pc.encode('MODE,MATERIAL')
        mv.uart.rx.extend(pc_message)
        self.assertEqual(mv.poll(), [['MODE', 'MATERIAL']])

    def test_openmv_to_python(self):
        mv = U.PacketSerial()
        pc = Protocol()
        mv.send('TARGET',7,1,165,118,21,12,-5,350,350,'S',92,1)
        self.assertEqual(pc.feed(mv.uart.tx),
                         ['TARGET,7,1,165,118,21,12,-5,350,350,S,92,1'])

    def test_reject_checksum(self):
        pc=Protocol()
        frame=bytearray(pc.encode('MODE,RING'))
        frame[-3] = ord('0') if frame[-3] != ord('0') else ord('1')
        self.assertEqual(pc.feed(frame), [])
        self.assertEqual(pc.bad_frames, 1)

    def test_reject_wrong_len(self):
        mv=U.PacketSerial()
        pc=Protocol()
        frame=pc.encode('MODE,QR').replace(b'7,MODE',b'8,MODE')
        mv.uart.rx.extend(frame)
        self.assertEqual(mv.poll(), [])


class MetricTests(unittest.TestCase):
    def setUp(self):
        self.saved = (C.FX_PX, C.FY_PX, C.CX_PX, C.CY_PX)
    def tearDown(self):
        C.FX_PX, C.FY_PX, C.CX_PX, C.CY_PX = self.saved

    def test_no_fake_metric(self):
        C.FX_PX = C.FY_PX = 0
        self.assertEqual(U.metric_from_circle(160,120,40,40), (0,0,0,0,0))

    def test_real_metric_optical_frame(self):
        C.FX_PX = C.FY_PX = 400
        C.CX_PX, C.CY_PX=160,120
        self.assertEqual(U.metric_from_circle(200,100,40,40),(40,-20,400,402,1))

    def test_no_material_when_not_locked(self):
        # Strict original lock: triangle hint requires 3 DIFFERENT colors.
        two=[(1,100.,100.,90,90,20,20,200),(2,200.,100.,190,90,20,20,200)]
        self.assertIsNone(U.triangle_hint(two))

    def test_trusted_disk_two_remaining_can_continue_after_occlusion(self):
        # Simulate a previously verified disk with only two materials left.
        # A new frame must still pass the REAL rim verification hook.
        gate = U.PersistentDiskGate()
        disk = (160.0, 120.0, 110.0)
        gate.trusted_disk = disk
        gate.disk = disk
        gate.locked = True
        old_ticks = getattr(U.time, 'ticks_ms', None)
        old_verify = U.local_verified_disk
        valid_frame = [True]
        try:
            U.time.ticks_ms = lambda: 1200
            U.local_verified_disk = lambda _gray, prev: prev if valid_frame[0] else None
            self.assertEqual(gate.update(None, [], 100), disk)
            valid_frame[0] = False
            self.assertIsNone(gate.update(None, [], 101))
            self.assertFalse(gate.locked)
            valid_frame[0] = True
            self.assertEqual(gate.update(None, [], 102), disk)
            self.assertTrue(gate.locked)
        finally:
            U.local_verified_disk = old_verify
            if old_ticks is None:
                delattr(U.time, 'ticks_ms')
            else:
                U.time.ticks_ms = old_ticks

    def test_duplicate_color_no_lock(self):
        fake_three=[(1,100.,100.,90,90,20,20,200),
                    (1,200.,100.,190,90,20,20,200),
                    (2,150.,200.,140,190,20,20,200)]
        self.assertIsNone(U.triangle_hint(fake_three))


if __name__ == '__main__':
    unittest.main(verbosity=2)
