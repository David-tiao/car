"""Pi5 session/action supervisor. Pure byte intents; never opens a tty or ROS publisher.
The caller must supply a verified map->odom conversion, live safety permission,
independent local MCU grant, and a current swept-envelope clearance checker.
"""
import math
from motion_link import (ARM, HEARTBEAT, TRANSLATE, TURN, STATUS, IDLE, MOVING,
                         DONE, FAULT, Frame, StreamDecoder, HostReview,
                         split_orthogonal_action)

HEARTBEAT_S = .10
REPLY_S = .12
MAX_RETRIES = 2
POS_TOL_MM = 8
YAW_TOL_MDEG = 3000

def angle_delta(a,b):
    return (a-b+180000)%360000-180000

def close_pose(p,q):
    return (p is not None and abs(p[0]-q[0])<=POS_TOL_MM and
            abs(p[1]-q[1])<=POS_TOL_MM and abs(angle_delta(p[2],q[2]))<=YAW_TOL_MDEG)

class PiSession:
    """One outstanding request; exactly one STOP intent on local failure.
    tick/receive use caller's monotonic seconds. No autonomous thread or port.
    Each completed chunk is checked against its independently computed endpoint.
    """
    def __init__(self,session,clearance):
        if not callable(clearance):raise ValueError('clearance checker required')
        self.host=HostReview(session);self.clearance=clearance;self.decoder=StreamDecoder()
        self.phase='NEW';self.reason=None;self.pose=None;self.action=None;self.chunk=None
        self.last=-1.;self.sent=None;self.first_sent=None;self.last_request=None;self.retries=0
        self.stop_wire=None;self.stop_emitted=False;self.completed=0;self.chunks=0;self.last_action_id=0
    def _fail(self,reason):
        if self.phase=='FAULT':return
        self.phase='FAULT';self.reason=reason;self.action=None;self.chunk=None
        self.stop_wire=self.host.stop()
    def _clock(self,now):
        if type(now) not in (int,float) or not math.isfinite(now) or now<0 or now<self.last:
            self._fail('INVALID_CLOCK');return False
        self.last=now;return True
    def start(self,local_grant_confirmed,tf_verified):
        if self.phase!='NEW':raise RuntimeError('not a new session')
        if local_grant_confirmed is not True or tf_verified is not True:
            raise ValueError('independent local grant and verified odom transform required')
        self.phase='ARMING'
    def queue_action(self,kind,start,goal):
        if self.phase!='IDLE' or self.action is not None or self.host.action is not None:
            raise RuntimeError('previous action not confirmed complete')
        split_orthogonal_action(start,goal,kind) # exact cardinal odom plan contract
        if not close_pose(self.pose,start):
            self._fail('START_POSE_MISMATCH');return False
        if kind=='MICRO_TRANSLATE' and close_pose(self.pose,goal):
            raise ValueError('micro displacement already within 8 mm protocol tolerance; recalibrate before use')
        self.action=(kind,tuple(start),tuple(goal));self.chunks=0;self.phase='ACTION'
        return True
    def _request(self,kind,args,now):
        wire=self.host.request(kind,*args)
        self.sent=self.first_sent=self.last_request=now;self.retries=0
        return wire
    def _next_chunk(self,now):
        kind,start,goal=self.action; p=self.pose; h=start[2]
        if close_pose(p,goal):
            self.action=None;self.phase='IDLE';self.completed+=1;return None
        if abs(angle_delta(p[2],h))>YAW_TOL_MDEG or self.chunks>=16:
            self._fail('HEADING_OR_CHUNK_LIMIT');return None
        if kind=='TURN_90':
            if self.chunks or abs(p[0]-start[0])>POS_TOL_MM or abs(p[1]-start[1])>POS_TOL_MM:
                self._fail('TURN_POSE_MISMATCH');return None
            k=TURN;args=(angle_delta(goal[2],h),);end=(p[0],p[1],goal[2])
        else:
            axis=0 if start[0]!=goal[0] else 1;cross=1-axis
            if abs(p[cross]-goal[cross])>POS_TOL_MM:
                self._fail('CROSS_TRACK_DRIFT');return None
            remaining=goal[axis]-p[axis]
            if abs(remaining)<=POS_TOL_MM:
                self._fail('UNRESOLVED_GOAL_ERROR');return None
            if kind=='MICRO_TRANSLATE' and abs(remaining)>20:
                self._fail('MICRO_RESIDUAL_LIMIT');return None
            part=max(-300,min(300,remaining));end=list(p);end[axis]+=part;end[2]=h;end=tuple(end)
            # Recompute the body-frame relative request from the latest measured odom.
            k,args=split_orthogonal_action((p[0],p[1],h),end,kind)[0]
        try:clear=self.clearance(tuple(p),end,kind) is True
        except Exception:clear=False
        if not clear:
            self._fail('CLEARANCE_REJECTED');return None
        self.chunk=end;self.chunks+=1
        return self._request(k,args,now)
    def tick(self,now,permitted):
        self._clock(now)
        if self.phase=='NEW':return None
        if permitted is not True:self._fail('SUPERVISOR_REVOKED')
        if self.phase=='FAULT':
            if self.stop_emitted:return None
            self.stop_emitted=True;return self.stop_wire
        if self.host.pending is not None:
            if now-self.sent<REPLY_S:return None
            if self.retries>=MAX_RETRIES:
                self._fail('REPLY_TIMEOUT');return self.tick(now,permitted)
            self.retries+=1;self.sent=now;return self.host.retry()
        if self.phase=='ARMING':return self._request(ARM,(),now)
        if self.action is not None and self.host.action is None and self.chunk is None:
            # The last accepted STATUS must remain fresh; refresh before a new chunk.
            if self.last_request is not None and now-self.last_request<HEARTBEAT_S:
                w=self._next_chunk(now)
                if w is not None:return w
                if self.phase=='FAULT':return self.tick(now,permitted)
        if self.last_request is None or now-self.last_request>=HEARTBEAT_S:
            return self._request(HEARTBEAT,(),now)
        return None
    def receive(self,data,now):
        if not self._clock(now) or self.phase in ('NEW','FAULT'):return False
        accepted=False
        for frame in self.decoder.feed(data):
            p=self.host.pending
            if p is None or frame.kind!=STATUS or frame.session!=self.host.session or frame.seq!=p.seq:continue
            if now-self.first_sent>=REPLY_S*(MAX_RETRIES+1):
                self._fail('LATE_REPLY');return False
            state,action=frame.args[:2]
            if p.kind==ARM and action!=0:
                self._fail('UNEXPECTED_ARM_ACTION');return False
            if p.kind==HEARTBEAT and self.host.action is None and (
                    action!=self.last_action_id or state!=(DONE if self.last_action_id else IDLE)):
                self._fail('UNEXPECTED_IDLE_STATUS');return False
            if not self.host.accept(frame.encode()):
                self._fail('STATUS_REJECTED');return False
            if p.kind in (TRANSLATE,TURN):self.last_action_id=p.seq
            accepted=True;self.pose=tuple(frame.args[2:5])
            if self.phase=='ARMING':self.phase='IDLE'
            if self.chunk is not None and self.host.state==DONE:
                if not close_pose(self.pose,self.chunk):
                    self._fail('DONE_POSE_MISMATCH');return False
                self.chunk=None
                if self.action is not None and close_pose(self.pose,self.action[2]):
                    self.action=None;self.phase='IDLE';self.completed+=1
        return accepted
