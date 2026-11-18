"""Host-side unit tests of OpenMV UART framing + distance math; no ROS2."""
import sys
import types
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src_openmv'))
for mod in ('image', 'display'):
    sys.modules[mod] = types.ModuleType(mod)
machine = types.ModuleType('machine')
class UART:
    def __init__(self, *args, **kwargs):
        self.rx = bytearray()
        self.tx = bytearray()
    def any(self): return len(self.rx)
    def read(self, n):
        data = self.rx[:n]
        del self.rx[:n]
        return data
    def write(self, data):
        self.tx.extend(data)
        return len(data)
machine.UART = UART
sys.modules['machine'] = machine
import config as C
import utils as U

def encode(seq, body):
    h = '%d,%d,%s' % (seq, len(body), body)
    chk = 0
    for c in h: chk ^= ord(c)
    return ('@%s*%02X\n' % (h, chk)).replace('\\n', '\n').encode('ascii')

class Tests(unittest.TestCase):
    def test_stm32_to_openmv(self):
        mv = U.PacketSerial()
        mv.uart.rx.extend(encode(2, 'MODE,MATERIAL'))
        self.assertEqual(mv.poll(), [['MODE','MATERIAL']])
    def test_openmv_to_stm32_framing(self):
        mv = U.PacketSerial()
        mv.send('TARGET', 7,1,165,118,21,0,0,0,0,'S',92,0)
        self.assertEqual(bytes(mv.uart.tx), encode(0,'TARGET,7,1,165,118,21,0,0,0,0,S,92,0'))
    def test_bad_crc_rejected(self):
        mv = U.PacketSerial()
        packet = bytearray(encode(2, 'PING'))
        packet[-3] = ord('0') if packet[-3]!=ord('0') else ord('1')
        mv.uart.rx.extend(packet)
        self.assertEqual(mv.poll(), [])
    def test_metric_invalid_until_calibration(self):
        old = (C.FX_PX, C.FY_PX)
        try:
            C.FX_PX = C.FY_PX = 0
            self.assertEqual(U.metric_from_circle(160,120,40,40),(0,0,0,0,0))
        finally:
            C.FX_PX, C.FY_PX = old
    def test_metric_projection(self):
        old = (C.FX_PX, C.FY_PX)
        try:
            C.FX_PX=C.FY_PX=400
            self.assertEqual(U.metric_from_circle(200,100,40,40),(40,-20,400,402,1))
        finally:
            C.FX_PX, C.FY_PX = old
    def test_three_distinct_colors_required_for_initial_gate(self):
        two=[(1,100.,100.,90,90,20,20,200),(2,200.,100.,190,90,20,20,200)]
        self.assertIsNone(U.triangle_hint(two))
    def test_one_missing_color_after_confirmed_disk(self):
        gate = U.PersistentDiskGate()
        disk = (160.0, 120.0, 110.0)
        gate.trusted_disk = disk
        gate.disk = disk
        gate.locked = True
        old_ticks=getattr(U.time,'ticks_ms', None)
        old_local=U.local_verified_disk
        try:
            U.time.ticks_ms=lambda:1000
            U.local_verified_disk=lambda gray, previous:previous
            self.assertEqual(gate.update(None,[],1), disk)
        finally:
            U.local_verified_disk=old_local
            if old_ticks is None: delattr(U.time,'ticks_ms')
            else: U.time.ticks_ms=old_ticks

if __name__=='__main__': unittest.main(verbosity=2)
