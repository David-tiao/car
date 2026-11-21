"""Run with MCU_TREE pointing at a checkout of architecture-v2, commit 1da165c."""
import ctypes
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "openmv_all"))
time.ticks_ms = lambda: int(time.monotonic() * 1000)
time.ticks_diff = lambda a, b: a-b


class UART:
    def __init__(self, *args, **kwargs):
        self.rx = bytearray()
        self.tx = bytearray()
    def any(self):
        return len(self.rx)
    def read(self, n):
        data = bytes(self.rx[:n])
        del self.rx[:n]
        return data
    def write(self, data):
        self.tx.extend(data[:7])   # intentionally short writes
        return min(7, len(data))


sys.modules["machine"] = SimpleNamespace(UART=UART)
import main as camera
from serial import V4Link


class MockImage:
    def to_grayscale(self, **kwargs):
        return self
    def find_qrcodes(self):
        return [SimpleNamespace(payload=lambda: '156+123+516+231')]


class Turntable(MockImage):
    def __init__(self):
        self.blocks = [(1,160,193), (5,96,83), (6,224,83)]

    def width(self):
        return 320

    def height(self):
        return 240

    def get_pixel(self, x, y):
        return 225 if (x-160)**2 + (y-120)**2 < 110**2 else 50

    def find_blobs(self, thresholds, roi=None, **kwargs):
        blobs = []
        for color, x, y in self.blocks:
            if roi is None:
                blobs.append(SimpleNamespace(code=1 << (color-1), rect=(x-12,y-12,25,25),
                                             pixels=440, cx=x, cy=y))
            elif roi[0] <= x < roi[0]+roi[2] and roi[1] <= y < roi[1]+roi[3]:
                blobs.append(SimpleNamespace(code=1 << (color-1), pixels=roi[2]*roi[3]))
        return blobs

    def find_circles(self, roi, **kwargs):
        x,y,w,h = roi
        return [SimpleNamespace(x=u, y=v, r=10, magnitude=2200) for _,u,v in self.blocks
                if x <= u < x+w and y <= v < y+h]


def wire(seq, body):
    head = ('%d,%d,%s' % (seq, len(body), body)).encode()
    checksum = 0
    for byte in head:
        checksum ^= byte
    return b'@' + head + ('*%02X\n' % checksum).encode()


class TestPeer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree = Path(os.environ["MCU_TREE"]).resolve()
        cls.temp = tempfile.TemporaryDirectory()
        lib = Path(cls.temp.name) / "peer.so"
        subprocess.run(["cc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                        "-I"+str(tree/"Hardware"), "-I"+str(tree/"tests/race/stubs"),
                        "-include", str(ROOT/"hal_tick.h"), str(ROOT/"peer.c"),
                        str(tree/"Hardware/vision.c"), "-o", str(lib)], check=True)
        cls.c = ctypes.CDLL(str(lib))
        cls.c.peer_why.restype = ctypes.c_char_p

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.c.peer_boot()
        self.link = V4Link()
        self.app = camera.ElectricalApp(self.link, SimpleNamespace(update=lambda *args: None))
        self.img = MockImage()
        self.now = 0
        self.flush_camera()
        self.assertTrue(self.c.peer_idle())

    def from_c(self):
        buf = ctypes.create_string_buffer(4096)
        n = self.c.peer_take(buf)
        self.inject(buf.raw[:n])
        return buf.raw[:n]

    def inject(self, data):
        for k in range(0, len(data), 3):
            self.link.uart.rx.extend(data[k:k+3])
            self.app.commands(self.now)

    def flush_camera(self):
        data = bytes(self.link.uart.tx)
        self.link.uart.tx.clear()
        self.c.peer_feed(data, len(data))
        return data

    def snap(self):
        self.now += 20
        self.app.frame(self.img, self.now)

    def test_session_qr(self):
        self.assertTrue(self.c.RaceVision_BeginScanSession(23))
        self.assertIn(b'R@', self.from_c())
        self.flush_camera()
        self.assertTrue(self.c.peer_running())
        for _ in range(3):
            self.snap()
        self.assertIn(b'QR,156+123+516+231,23', self.flush_camera())
        code = ctypes.create_string_buffer(16)
        self.assertTrue(self.c.peer_qr(code))
        self.assertEqual(code.value, b'156+123+516+231')
        self.assertTrue(self.c.RaceVision_AckQr(22))
        self.from_c()
        self.assertEqual(self.app.state, 'QR')
        self.now += 300
        self.snap()
        self.assertIn(b'QR,156+123+516+231,23', self.flush_camera())
        self.assertTrue(self.c.RaceVision_AckQr(23))
        self.from_c()
        self.assertIn(b'DONE,QR', self.flush_camera())
        self.assertTrue(self.c.peer_idle())
        self.assertEqual(self.app.state, 'IDLE')

    def test_ring_step_and_finish(self):
        self.assertTrue(self.c.peer_ring(2))
        self.from_c()
        self.flush_camera()
        self.assertTrue(self.c.peer_running())
        center = [172, 115]
        self.app.stations.empty_ring = lambda img, target: tuple(center) if target == 2 else None
        for _ in range(3): self.snap()
        self.assertIn(b'POS,RING,2,12,-5,172,115,1,0', self.flush_camera())
        self.assertTrue(self.c.peer_pose(2, 12, -5, 172, 115))
        self.snap()
        self.assertEqual(self.flush_camera(), b'')
        self.assertTrue(self.c.peer_moved())
        self.from_c()
        center[:] = [161, 121]
        for i in range(1, 5):
            for _ in range(3): self.snap()
            self.assertIn(('POS,RING,2,1,1,161,121,1,%d' % i).encode(), self.flush_camera())
            self.assertTrue(self.c.peer_stable())
            self.from_c()
        self.flush_camera()
        self.assertTrue(self.c.peer_ring_success())

    def start_area(self):
        self.assertTrue(self.c.peer_job(40))
        self.from_c()
        self.flush_camera()
        self.assertTrue(self.c.peer_running())
        self.app.source.disk_sample = lambda img: ((160, 120, 105), None, [])
        self.app.source.disk.locked = True
        self.snap()

    def test_area_material(self):
        self.start_area()
        self.assertIn(b'AREA_OK,160,120,105', self.flush_camera())
        self.assertTrue(self.c.peer_area())
        self.assertTrue(self.c.peer_idle())
        self.assertTrue(self.c.peer_job(41))
        self.from_c()
        self.flush_camera()
        self.app.source.observe = lambda *args: (180, 186, 19, 1)
        self.snap()
        self.flush_camera()
        self.assertTrue(self.c.peer_pose(0, 20, -4, 180, 186))
        self.app.source.observe = lambda *args: (160, 190, 19, 1)
        self.snap()
        self.assertIn(b'ARRIVED,MATERIAL,0,0,0', self.flush_camera())
        self.assertTrue(self.c.peer_at_target())
        self.assertTrue(self.c.peer_idle())

    def test_material_pipeline_on_synthetic_scene(self):
        self.img = Turntable()
        third = self.img.blocks.pop()
        self.assertTrue(self.c.peer_job(40))
        self.from_c(); self.flush_camera()
        for _ in range(45):
            self.snap(); self.flush_camera()
        self.assertEqual(self.app.state, 'AREA')
        self.img.blocks.append(third)
        for _ in range(45):
            self.snap(); self.flush_camera()
            if self.c.peer_idle():
                break
        self.assertEqual(self.app.state, 'IDLE')
        self.assertTrue(self.c.peer_job(41))
        self.from_c(); self.flush_camera()
        for _ in range(100):
            self.snap(); self.flush_camera()
            if self.c.peer_at_target():
                break
        self.assertTrue(self.c.peer_at_target())
        self.assertTrue(self.c.peer_idle())

    def test_area_separate_delivery_requires_peer_wait_for_idle(self):
        self.start_area()
        area, idle = bytes(self.link.uart.tx).splitlines(True)
        self.c.peer_feed(area, len(area))
        self.assertTrue(self.c.peer_area())
        self.assertFalse(self.c.peer_job(41))
        self.c.peer_feed(idle, len(idle))
        self.assertTrue(self.c.peer_job(41))

    def test_cancel_reset_and_restart(self):
        self.assertTrue(self.c.RaceVision_BeginScanSession(8)); self.from_c(); self.flush_camera()
        self.assertTrue(self.c.peer_cancel()); self.from_c(); self.flush_camera()
        self.assertTrue(self.c.peer_idle())
        self.assertTrue(self.c.peer_reset()); self.from_c(); self.flush_camera()
        self.assertTrue(self.c.peer_idle())
        self.c.peer_boot()
        self.assertTrue(self.c.RaceVision_BeginScanSession(1))
        self.from_c(); self.flush_camera()
        self.assertEqual(self.app.session, 1)
        self.assertTrue(self.c.peer_running())

    def test_peer_reboot_reusing_session_must_rescan(self):
        self.assertTrue(self.c.RaceVision_BeginScanSession(1))
        self.from_c(); self.flush_camera()
        for _ in range(3): self.snap()
        self.assertIsNotNone(self.app.qr)
        self.flush_camera()
        self.c.peer_boot()
        self.assertTrue(self.c.RaceVision_BeginScanSession(1))
        self.from_c()
        self.assertEqual(self.app.session, 1)
        self.assertIsNone(self.app.qr)
        self.assertEqual(self.app.board.qr_hits, 0)

    def test_qr_without_session_and_bare_r(self):
        self.inject(b'R')
        self.assertEqual(self.app.state, 'IDLE')
        self.assertTrue(self.c.RaceVision_BeginScanSession(0))
        self.from_c(); self.flush_camera()
        for _ in range(3): self.snap()
        self.assertIn(b'QR,156+123+516+231*', self.flush_camera())
        self.assertTrue(self.c.RaceVision_AckQr(0))
        self.from_c(); self.flush_camera()
        self.assertTrue(self.c.peer_idle())

    def test_bad_crc_length_replay_and_wrap(self):
        data = bytearray(wire(1, 'RESET')); data[-3] ^= 1
        self.inject(bytes(data) + b'@1,23,RESET*00\n' + b'@' + b'z'*202 + b'\n')
        self.assertFalse(self.link.uart.tx)
        self.inject(wire(65535, 'STATUS'))
        self.assertIn(b'READY,IDLE', self.flush_camera())
        self.inject(wire(65535, 'RESET'))
        self.assertFalse(self.link.uart.tx)
        self.inject(wire(0, 'STATUS'))
        self.assertIn(b'READY,IDLE', self.flush_camera())

    def test_stale_step_or_arrival_does_not_advance(self):
        self.assertTrue(self.c.peer_ring(1)); self.from_c(); self.flush_camera()
        self.app.stations.empty_ring = lambda *args: (162,119)
        for _ in range(3): self.snap()
        self.flush_camera()
        self.inject(wire(2,'STEP_DONE,8,1') + wire(3,'ARRIVED,RING,1,2,-1,9,4'))
        self.assertTrue(self.app.wait_step)
        self.assertEqual((self.app.state,self.app.step), ('RING',0))

    def test_unwired_job_cannot_claim_pick(self):
        self.assertTrue(self.c.peer_job(42)); self.from_c(); self.flush_camera()
        self.assertTrue(self.c.peer_error())
        self.assertEqual(self.c.peer_why(), b'UNSUPPORTED_JOB')


if __name__ == '__main__':
    unittest.main(verbosity=2)
