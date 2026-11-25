"""Exercise the four deployed files with scripted camera observations on a PC."""
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

CAMERA = Path(__file__).resolve().parents[1] / 'openmv_all'
sys.path.insert(0, str(CAMERA))
with patch.dict(sys.modules, {'machine': types.SimpleNamespace(UART=Mock())}):
    import main as camera


class RaceCameraTest(unittest.TestCase):
    def setUp(self):
        self.link = Mock()
        self.app = camera.ElectricalApp(self.link, Mock())
        self.app.command(('SCAN_BEGIN', '1', '19'))
        self.app.board.load(['156', '123', '516', '231'])
        self.app.qr = '+'.join(self.app.board.groups)
        self.app.command(('ACK', 'QR', '19'))
        self.detector = Mock()
        self.app.stations = self.detector
        self.window = (160, 120, 18)
        self.source = Mock()
        self.source.observe.return_value = (*self.pick_point, 10, 1)
        for obj, attr, value in (
            (time, 'ticks_diff', lambda a, b: a-b),
            (camera, 'SourceStation', Mock(return_value=self.source)),
            (camera.C, 'TRAY_EMPTY_LAB', (50, 90, -60, -20, 20, 60)),
            (camera.C, 'TRAY_WINDOWS', {key: (self.window,)*3 for key in camera.C.TRAY_WINDOWS}),
            (camera.C, 'HOME_WINDOWS', {1:self.window, 2:self.window}),
            (camera.C, 'STACK_WINDOWS', {i: ((50, 95, 14), (50, 45, 14)) for i in (1, 2, 3)}),
        ):
            item = patch.object(obj, attr, value, create=True)
            item.start()
            self.addCleanup(item.stop)
        self.new_source = camera.SourceStation
        self.now = 0

    @property
    def pick_point(self):
        return camera.C.PICK_TARGET_X, camera.C.PICK_TARGET_Y

    def frames(self, count):
        for _ in range(count):
            self.now += 60
            self.app.frame(None, self.now)

    def check(self, phase):
        op, _, color, ring = self.app.board.current()
        if op == 'HOME':
            ring = 2
        return 'CHECK', '19', str(self.app.board.step), phase, str(color), str(ring)

    def test_full_example(self):
        for step in range(1, 26):
            op, station, color, ring = self.app.board.current()
            for phase in ('B', 'A'):
                before = phase == 'B'
                expected = color if (op == 'PICK') != before else 0
                self.detector.tray_evidence.side_effect = lambda img, window, col: col == expected
                point = (camera.C.RING_TARGET_X, camera.C.RING_TARGET_Y)
                empty = (op == 'PLACE') == before
                self.detector.empty_ring.return_value = point if empty else None
                self.detector.occupied_ring.return_value = None if empty else point
                if station == 'STORE' and op == 'STACK':
                    target = camera.C.STACK_WINDOWS[ring][int(not before)]
                    self.detector.face_in_window.side_effect = lambda img, w, c: point if w == target and c == color else None
                self.detector.home_patch.return_value = point
                cmd = self.check(phase)
                self.app.command(cmd)
                self.frames(1)
                self.app.command(cmd)  # MCU timeout/retry must preserve consecutive frames.
                self.frames(camera.C.STATION_CONFIRM_FRAMES-1)
                self.link.send.assert_called_with('CAN' if before else 'DID', 19, step)
                if station == 'RAW' and before:
                    self.assertEqual(self.source.observe.call_args.args[1], color)
                if not before:
                    self.app.command(cmd)  # Lost DID, no duplicate count.
                    self.link.send.assert_called_with('DID', 19, step)
                    self.assertEqual(self.app.board.step, step+1)
        self.assertEqual((self.app.board.picks, self.app.board.places), (12, 12))
        self.assertEqual(self.new_source.call_count, 2)  # A fresh disk at each batch only.

    def test_reject_stale_wrong_or_unconfirmed(self):
        for cmd in (('CHECK','18','1','B','1','0'), ('CHECK','19','2','B','5','0'),
                    ('CHECK','19','1','B','5','0'), self.check('A')):
            with self.assertRaises(ValueError):
                self.app.command(cmd)
        cmd = self.check('B')
        with patch.object(camera.C, 'TRAY_EMPTY_LAB', None), self.assertRaisesRegex(ValueError,'CAL_TRAY'):
            self.app.command(cmd)
        self.app.command(cmd)
        self.source.observe.return_value = None
        self.frames(8)
        self.assertNotIn(unittest.mock.call('CAN', 19, 1), self.link.send.call_args_list)
        self.source.observe.return_value = (*self.pick_point, 10, 1)
        self.frames(3)
        self.app.command(self.check('A'))
        self.detector.tray_evidence.return_value = False
        self.frames(8)
        self.assertEqual((self.app.board.step, self.app.board.picks), (1, 0))

    def test_v4_frame_fragment_duplicate_and_bad_checksum(self):
        link = camera.V4Link()
        packet = link.encode(('CHECK',19,1,'B',1,0))
        data = bytearray(packet[:6])
        link.uart.any.side_effect = lambda: len(data)
        def read(n):
            chunk = bytes(data[:n])
            del data[:n]
            return chunk
        link.uart.read.side_effect = read
        self.assertEqual(link.poll(), [])
        data.extend(packet[6:]+packet)
        self.assertEqual(link.poll(), [('CHECK','19','1','B','1','0')])
        bad = link.encode(('CHECK',19,1,'A',1,0)).replace(b',A,', b',B,')
        data.extend(bad)
        self.assertEqual(link.poll(), [])

    def test_source_outside_gripper(self):
        self.app.command(self.check('B'))
        self.source.observe.return_value = (8, 8, 11, 2)
        self.frames(8)
        self.assertIsNone(self.app.prepared)


if __name__ == '__main__':
    unittest.main()
