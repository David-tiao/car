"""Portable smoke test of OpenMV math, without pretending to test hardware."""
import importlib.util
import sys
import types
from pathlib import Path
root = Path(__file__).resolve().parent.parent / 'OpenMV'
sys.path.insert(0, str(root))
for name in ('image', 'display'):
    sys.modules[name] = types.ModuleType(name)
m = types.ModuleType('machine')
m.UART = object
sys.modules['machine'] = m
spec = importlib.util.spec_from_file_location('openmv_v4_utils', root / 'utils.py')
u = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u)
slot = u.nearest_pick_slot((160., 120., 110.))
assert slot == (160, 193), slot
class KF:
    def __init__(self,x): self.x = x
class T:
    kx, ky = KF(160.), KF(193.)
assert u.in_pick_slot(T(),slot)
assert u.parse_code('452+321+254+312') == ['452','321','254','312']
assert u.parse_code('452+321+123+312') is None
# Recognition must not rescan arbitrary circles: recover only at a recorded pose.
gate = u.DiskGate()
gate.disk = (160.,120.,110.)
gate.locked = True
u.disk_rim_score = lambda *args: -1
u.disk_core_ok = lambda *args: True
for frame in (1,2,3):
    assert gate.update(None,[],frame) is None
assert not gate.locked and gate.recovery_anchor == (160.,120.,110.)
u.detect_disk = lambda *args: None
u.disk_rim_score = lambda gray,cx,cy,r: (18 if abs(cx-160)<2 and abs(cy-120)<2 and abs(r-110)<2 else -1)
for frame in (4,8,12,16):
    assert gate.update(None,[],frame) is None
assert gate.update(None,[],20) is not None and gate.locked
print('PASS OpenMV fixed slot, QR consistency, disk reacquisition with missing materials')
