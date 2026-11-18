"""Pure-Python flow test using fake OpenMV board objects; not a camera accuracy test."""
import importlib.util
import sys
import types
from pathlib import Path

root = Path(__file__).resolve().parent.parent / 'OpenMV'
sys.path.insert(0, str(root))
for name in ('image', 'display', 'csi'):
    sys.modules[name] = types.ModuleType(name)
m = types.ModuleType('machine')
m.UART = object
sys.modules['machine'] = m
spec = importlib.util.spec_from_file_location('minimal_main', root / 'main.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.time.ticks_diff = lambda a,b: a-b

class FakeUART:
    def __init__(self):
        self.sent = []
        self.commands = []
        self.uart = self
    def poll(self):
        result, self.commands = self.commands, []
        return result
    def send(self, packet):
        self.sent.append(packet)
    def write(self, packet):
        self.sent.append(packet.decode('ascii'))
    def push(self, *commands):
        self.commands.extend(commands)

class FakeLCD:
    def __init__(self): self.updates = 0
    def update(self, board): self.updates += 1

class FakeImg:
    def to_grayscale(self, copy=True): return object()
    def draw_circle(self, *args, **kwargs): pass
    def draw_string(self, *args, **kwargs): pass
    def draw_cross(self, *args, **kwargs): pass

class Track:
    def __init__(self, color, x=160, y=193):
        self.label = color
        self.id = color
        self.kx = types.SimpleNamespace(x=float(x))
        self.ky = types.SimpleNamespace(x=float(y))
        self.r = 12
        self.seen = True
        self.hits = 5
        self.state = 'S'

class FakeTracker:
    def __init__(self): self.items = []
    def update(self, detection, now): return []
    def clear(self): self.items = []

app = module.App.__new__(module.App)
app.state = 'MATERIAL'
app.board = module.U.TaskBoard()
assert app.board.accept('452+321+254+312')
app.lcd = FakeLCD()
app.uart = FakeUART()
app.disk = types.SimpleNamespace(locked=True)
app.material = FakeTracker()
app.frame = 0
app.item = 0
app.disk_prev = True
app.pick_slot = (160,193)
app.armed = False
app.taking = False
app.target_id = None
app.slot_since = 0
app.last_slot_tx = 0
app.pending = None
app.pending_ms = 0
app.retries = 0
app.update_disk = lambda img,gray,blobs,now: (160.,120.,110.)
module.U.color_blobs = lambda img: []
module.U.material_candidates = lambda *args: []
# First color = 4. If color 5 is in the claw slot, picking must NOT fire.
app.uart.push('#ARM,1')
app.process_commands()
assert app.armed and app.uart.sent[-1] == '#ARM_OK,1\n'
app.material.items = [Track(5)]
app.update_material(FakeImg(),100)
app.update_material(FakeImg(),300)
assert app.state == 'MATERIAL' and not any(s.startswith('#OBJ') for s in app.uart.sent)
# Color 4 arrives; only after continuous stability inside the station.
app.material.items = [Track(4)]
app.update_material(FakeImg(),400)
assert app.state == 'MATERIAL'
app.update_material(FakeImg(),600)
assert app.state == 'WAIT_ACK' and app.uart.sent[-1].startswith('#OBJ,4,')
# Premature K must not advance the task.
app.uart.push('K')
app.process_commands()
assert app.item == 0 and app.state == 'WAIT_ACK'
# TAKE confirmation binds the mechanical action to exactly one task.
app.uart.push('#TAKE')
app.process_commands()
assert app.taking and app.uart.sent[-1] == '#TAKE_OK\n'
app.uart.push('K','K')
app.process_commands()
assert app.item == 1 and app.state == 'MATERIAL'
# First object not enough; colour 5 is now expected and can be processed.
app.uart.push('#ARM,1')
app.process_commands()
app.material.items = [Track(5)]
app.update_material(FakeImg(),700)
app.update_material(FakeImg(),900)
assert app.state == 'WAIT_ACK' and app.uart.sent[-1].startswith('#OBJ,5,')
# last two K/TAKE sequences complete first batch only.
app.uart.push('#TAKE','K')
app.process_commands()
assert app.item == 2 and app.state == 'MATERIAL'
app.uart.push('#ARM,1')
app.process_commands()
app.material.items = [Track(2)]
app.update_material(FakeImg(),1000)
app.update_material(FakeImg(),1200)
assert app.state == 'WAIT_ACK'
app.uart.push('#TAKE','K','#GROUP_DONE,1')
app.process_commands()
assert app.item == 3 and app.state == 'DONE' and app.board.completed == 1
assert app.board.remaining_text() == '321+254+312'
print('PASS OpenMV order gate / single pick confirmation / first-batch halt')
