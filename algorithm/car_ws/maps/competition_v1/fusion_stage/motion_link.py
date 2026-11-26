"""Offline reference codec. Never opens a serial port or publishes ROS commands."""
import binascii
import struct
from dataclasses import dataclass

ARM, HEARTBEAT, TRANSLATE, TURN, STOP, STATUS = 1, 2, 3, 4, 5, 128
DISARMED, IDLE, MOVING, DONE, FAULT = range(5)
OK, REJECT, TIMEOUT, FEEDBACK, INTERLOCK, STOPPED, BUSY, BADSEQ, BADARG, MOVE_TIMEOUT = range(10)
MAGIC = b'M2\x01'
BODY = struct.Struct('<2sBBII6i')
SIZE = BODY.size + 2

@dataclass(frozen=True)
class Frame:
    kind: int
    session: int
    seq: int
    args: tuple = (0,)*6

    def encode(self):
        if (type(self.kind) is not int or self.kind not in (ARM,HEARTBEAT,TRANSLATE,TURN,STOP,STATUS)
                or type(self.session) is not int or not 1 <= self.session <= 0xffffffff
                or type(self.seq) is not int or not 1 <= self.seq <= 0x7fffffff
                or len(self.args) != 6 or any(type(a) is not int or not -2**31 <= a < 2**31 for a in self.args)):
            raise ValueError('invalid frame')
        body = BODY.pack(b'M2',1,self.kind,self.session,self.seq,*self.args)
        return body+struct.pack('<H',binascii.crc_hqx(body,0xffff))

    @classmethod
    def decode(cls, wire):
        if len(wire)!=SIZE or wire[:3]!=MAGIC or binascii.crc_hqx(wire[:-2],0xffff)!=struct.unpack('<H',wire[-2:])[0]:
            raise ValueError('bad frame')
        _,_,kind,session,seq,*args=BODY.unpack(wire[:-2])
        frame=cls(kind,session,seq,tuple(args)); frame.encode(); return frame

class StreamDecoder:
    """Bounded incremental decoder: junk, fragmentation, coalescing and bad CRC."""
    def __init__(self): self.buf=bytearray()
    def feed(self, data):
        out=[]
        for b in data:
            self.buf.append(b)
            while self.buf and not (MAGIC.startswith(self.buf) if len(self.buf)<3 else self.buf[:3]==MAGIC):
                del self.buf[0]
            if len(self.buf)==SIZE:
                try: out.append(Frame.decode(self.buf));self.buf.clear()
                except ValueError:
                    del self.buf[0]
                    while self.buf and not (MAGIC.startswith(self.buf) if len(self.buf)<3 else self.buf[:3]==MAGIC):del self.buf[0]
        return out

class HostReview:
    """One outstanding frame, exact retries, explicit action DONE correlation.
    No transport or automatic actuation. Caller supplies monotonic local clock.
    """
    def __init__(self, session):
        Frame(ARM,session,1).encode()
        self.session=session;self.seq=0;self.pending=None;self.action=None;self.state=DISARMED;self.failed=False
    def request(self, kind, *args):
        if self.failed or self.pending is not None:raise RuntimeError('session blocked or request pending')
        if kind not in (ARM,HEARTBEAT,TRANSLATE,TURN,STOP) or len(args)>6:raise ValueError('invalid request')
        if kind in (TRANSLATE,TURN) and (self.action is not None or self.state not in (IDLE,DONE)):
            raise RuntimeError('previous action not complete')
        if kind==TRANSLATE and (len(args)!=2 or any(type(a) is not int or abs(a)>300 for a in args) or (args[0]==0)==(args[1]==0)):
            raise ValueError('one nonzero body axis, max 300 mm')
        if kind==TURN and (len(args)!=1 or args[0] not in (-90000,90000)):raise ValueError('quarter turn only')
        if kind in (ARM,HEARTBEAT,STOP) and args:raise ValueError('unexpected args')
        frame=Frame(kind,self.session,self.seq+1,tuple(args)+(0,)*(6-len(args)))
        wire=frame.encode();self.seq=frame.seq;self.pending=frame;return wire
    def retry(self):
        if self.failed or self.pending is None:raise RuntimeError('no pending frame')
        return self.pending.encode()
    def stop(self):
        # Emergency STOP is separate from ordinary outstanding requests.
        # This is only an encoded test frame; it does not send bytes.
        seq=min(self.seq+1,0x7fffffff);self.failed=True;self.pending=None
        return Frame(STOP,self.session,max(1,seq)).encode()
    def accept(self, wire):
        f=Frame.decode(wire)
        if self.pending is None or f.kind!=STATUS or f.session!=self.session or f.seq!=self.pending.seq:return False
        state,action,x,y,yaw,reason=f.args
        p=self.pending; self.pending=None
        valid_pose=abs(x)<=10000 and abs(y)<=10000 and -180000<=yaw<180000
        if state not in range(5) or reason!=OK or state==FAULT or not valid_pose:
            self.failed=True;return False
        if p.kind==ARM and state!=IDLE:self.failed=True;return False
        if p.kind in (TRANSLATE,TURN):
            if action!=p.seq or state not in (MOVING,DONE):self.failed=True;return False
            self.action=p.seq
        if self.action is not None:
            if action!=self.action or state not in (MOVING,DONE):self.failed=True;return False
            if state==DONE:self.action=None
        self.state=state;return True

def split_orthogonal_action(start, goal, kind):
    """Translate an *odom-frame* integer pose action into bounded body commands.
    Input: (x_mm,y_mm,yaw_mdeg), exact cardinal headings. Caller must transform
    map to odom with verified TF first. No assumption map == odom. Each chunk
    requires independent DONE before the next; not an automatic sender.
    """
    headings=(0,90000,-180000,-90000)
    for pose in (start,goal):
        if len(pose)!=3 or any(type(v) is not int for v in pose) or any(abs(v)>10000 for v in pose[:2]) or pose[2] not in headings:
            raise ValueError('invalid cardinal odom pose')
    x,y,h=start;gx,gy,gh=goal
    dx,dy=gx-x,gy-y
    if kind=='TURN_90':
        delta=(gh-h+180000)%360000-180000
        if dx or dy or abs(delta)!=90000:raise ValueError('not a stationary quarter turn')
        return [(TURN,(delta,))]
    if kind not in ('TRANSLATE','MICRO_TRANSLATE') or h!=gh or (dx==0)==(dy==0):raise ValueError('not an axis translation')
    bx,by={0:(dx,dy),90000:(dy,-dx),-180000:(-dx,-dy),-90000:(-dy,dx)}[h]
    if kind=='MICRO_TRANSLATE' and abs(bx)+abs(by)>20:raise ValueError('micro step exceeds 20 mm')
    axis=0 if bx else 1;remaining=bx or by;out=[]
    while remaining:
        part=max(-300,min(300,remaining));a=[0,0];a[axis]=part;out.append((TRANSLATE,tuple(a)));remaining-=part
    return out
