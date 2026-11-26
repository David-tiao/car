"""Offline segment handshake, no serial, no hardware authorization.
Coordinates SI, map and odom poses are explicitly distinct. Planner is external.
"""
from dataclasses import dataclass, asdict
import math
import re


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    def __post_init__(self):
        if not all(type(v) in (float, int) and math.isfinite(v) for v in (self.x,self.y,self.yaw)):
            raise ValueError('non-finite pose')

    def compose(self, b):
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        return Pose(self.x+c*b.x-s*b.y,self.y+s*b.x+c*b.y,wrap(self.yaw+b.yaw))

    def inverse(self):
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        return Pose(-c*self.x-s*self.y,s*self.x-c*self.y,wrap(-self.yaw))


@dataclass(frozen=True)
class Limits:
    # Explicitly supplied by caller; demo values are not hardware calibration.
    pos: float
    yaw: float
    speed: float
    angular_speed: float
    max_deviation: float
    fresh_ms: int
    ack_ms: int
    result_ms: int
    settle_ms: int
    vision_ms: int
    stable_samples: int
    def __post_init__(self):
        for k,v in asdict(self).items():
            if type(v) not in (int,float) or not math.isfinite(v) or v<=0:
                raise ValueError(k)
        for k in ('fresh_ms','ack_ms','result_ms','settle_ms','vision_ms','stable_samples'):
            if type(getattr(self,k)) is not int: raise ValueError(k)
        if self.stable_samples<2 or self.max_deviation<self.pos: raise ValueError('limits')


@dataclass(frozen=True)
class Sample:
    source_time_ms: int
    pose: Pose
    vx: float
    vy: float
    wz: float
    target_reached: bool
    imu_valid: bool
    def __post_init__(self):
        if type(self.source_time_ms) is not int or not 0<=self.source_time_ms<=0xffffffff: raise ValueError('source time')
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (self.vx,self.vy,self.wz)): raise ValueError('velocity')


class Executor:
    """A single in-flight SIM_ONLY position intent; never sends a STOP frame either.
    The external planner must validate footprint, free space and corrections.
    A plan is a list of map-frame Pose waypoints, starting at actual map pose.
    Map identity is opaque and fixed throughout this session.
    """
    def __init__(self, map_id, map_to_odom, limits, success_status, first_id=1):
        if not isinstance(map_id,str) or not map_id or type(success_status) is not int or not 0<=success_status<=255: raise ValueError('configuration')
        if type(first_id) is not int or not 1<=first_id<=0xffffffff: raise ValueError('cmd id')
        self.map_id=map_id; self.tf=map_to_odom; self.l=limits; self.ok=success_status
        self.next_id=first_id; self.state='IDLE'; self.reason=''; self.now=-1
        self.sample=None; self.sample_at=None; self.link_at=None; self.session=None
        self.route=(); self.index=0; self.pending=None; self.deadline=None; self.stable=0
        self.goal=None; self.stage=None; self.stop_pose=None; self.vision_token=None
        self.complete_source=None; self.last_feedback_source=None; self.settled=[]; self.task_code=None

    def _clock(self, now):
        if type(now) is not int or now<0 or now<self.now: raise ValueError('monotonic ms')
        self.now=now

    def _fault(self, why):
        self.state='FAULT'; self.reason=why; self.deadline=None; self.pending=None
        self.stable=0

    def _fresh(self, now):
        return (self.sample is not None and self.sample.imu_valid is True and
                self.link_at is not None and now-self.link_at<=self.l.fresh_ms and
                now-self.sample_at<=self.l.fresh_ms)

    def _still(self,s):
        return math.hypot(s.vx,s.vy)<=self.l.speed and abs(s.wz)<=self.l.angular_speed

    def tick(self, now):
        self._clock(now)
        if self.state not in ('IDLE','FAULT','DONE'):
            if not self._fresh(now): self._fault('STALE_OR_INVALID_TELEMETRY')
            elif self.deadline is not None and now>=self.deadline: self._fault('TIMEOUT')
        return self.state

    def link(self, now, session, connected, heartbeat_fresh):
        self.tick(now)
        if not connected or not heartbeat_fresh or type(session) is not int or not 1<=session<=0xffffffff:
            self.link_at=None
            if self.state not in ('IDLE','FAULT','DONE'): self._fault('LINK_LOST')
            return
        if self.session is not None and session!=self.session:
            self._fault('PEER_SESSION_CHANGED'); return
        self.session=session; self.link_at=now

    def telemetry(self, now, s):
        self.tick(now)
        if self.state=='FAULT': return False
        if self.sample is not None and s.source_time_ms<=self.sample.source_time_ms:
            # Fail safe on reset/wrap/reordered samples; new execution session required.
            return False
        self.sample=s; self.sample_at=now
        if self.state=='WAIT_SETTLE':
            if s.source_time_ms<=self.complete_source: return False
            actual=self.tf.compose(s.pose)
            target=self.pending['target_map']
            err=math.hypot(actual.x-target.x,actual.y-target.y)
            if err>self.l.max_deviation:
                self._fault('EXCESS_POSITION_DEVIATION'); return False
            good=(s.imu_valid is True and s.target_reached is True and self._still(s) and
                  err<=self.l.pos and abs(wrap(actual.yaw-target.yaw))<=self.l.yaw)
            self.stable=self.stable+1 if good else 0
            if self.stable>=self.l.stable_samples:
                self.settled.append(actual); self.stop_pose=actual; self.index+=1
                token=self.pending['cmd_id']; self.pending=None; self.stable=0
                if self.index==len(self.route):
                    self.state='WAIT_VISION'; self.deadline=now+self.l.vision_ms; self.vision_token=token
                else:
                    # Next axis needs planning from actual pose, never snap to ideal waypoint.
                    self.state='NEED_REPLAN'; self.deadline=None
        elif self.state in ('WAIT_VISION','NEED_REPLAN','READY') and (not self._still(s) or s.imu_valid is not True):
            self._fault('UNEXPECTED_MOTION_OR_INVALID_IMU')
        return True

    def load_route(self, now, map_id, stage, goal, route, corridor_check):
        """External corridor_check(a,b) must certify each segment including footprint.
        In this review, it is a simulator fixture, NOT a hardware safety permit.
        """
        self.tick(now)
        if self.state not in ('IDLE','NEED_REPLAN'): raise ValueError('not ready to plan')
        if map_id!=self.map_id: self._fault('MAP_CHANGED'); return False
        if not self._fresh(now) or not self._still(self.sample): self._fault('START_NOT_STABLE'); return False
        expected_stage = ('QR' if self.stage is None else
                          'RAW' if self.stage=='QR' and self.goal is None else self.stage)
        if stage!=expected_stage: raise ValueError('stage')
        if self.state=='NEED_REPLAN' and self.goal is not None and (stage!=self.stage or goal!=self.goal):
            raise ValueError('cannot change unfinished stage goal')
        actual=self.tf.compose(self.sample.pose); route=tuple(route)
        if len(route)<2 or route[0]!=actual or route[-1]!=goal: raise ValueError('route must start at measured pose and end at goal')
        for a,b in zip(route,route[1:]):
            dx,dy=abs(a.x-b.x),abs(a.y-b.y)
            if min(dx,dy)>1e-9 or max(dx,dy)<=1e-9 or abs(wrap(a.yaw-b.yaw))>1e-9:
                raise ValueError('only nonzero orthogonal translation at fixed yaw')
            if corridor_check(a,b) is not True: self._fault('CORRIDOR_NOT_CERTIFIED'); return False
        self.route=route; self.index=1; self.stage=stage; self.goal=goal; self.state='READY'; self.deadline=None
        return True

    def issue(self, now):
        self.tick(now)
        if self.state!='READY': return None
        if self.tf.compose(self.sample.pose)!=self.route[self.index-1]:
            self.state='NEED_REPLAN'; return None
        if self.next_id>0xffffffff: self._fault('CMD_ID_EXHAUSTED'); return None
        target=self.route[self.index]; odom=self.tf.inverse().compose(target)
        self.pending={'cmd_id':self.next_id,'target_map':target,'target_odom':odom,
                      'map_id':self.map_id,'stage':self.stage,'source_floor':self.sample.source_time_ms}
        self.next_id+=1; self.state='WAIT_ACK'; self.deadline=now+self.l.ack_ms
        self.last_feedback_source=None
        return dict(self.pending, interface='SIM_ONLY', hardware_authorized=False, command_type=0x83)

    def feedback(self, now, cmd_id, event_type, source_time_ms, status_code=0, original_command_type=0):
        self.tick(now)
        if self.state not in ('WAIT_ACK','WAIT_RESULT','WAIT_SETTLE') or self.pending is None or cmd_id!=self.pending['cmd_id']: return False
        if type(source_time_ms) is not int or not self.pending['source_floor']<=source_time_ms<=0xffffffff: return False
        if self.last_feedback_source is not None and source_time_ms<self.last_feedback_source: return False
        if event_type in (0x91,0x95): self._fault('COMMAND_REJECTED_OR_FAILED'); return True
        if event_type==0x90 and self.state=='WAIT_ACK' and original_command_type==0x83:
            if status_code!=self.ok: self._fault('ACK_STATUS_NOT_SUCCESS'); return True
            self.last_feedback_source=source_time_ms; self.state='WAIT_RESULT'; self.deadline=now+self.l.result_ms; return True
        if event_type==0x94 and self.state=='WAIT_RESULT':
            if status_code!=self.ok: self._fault('RESULT_STATUS_NOT_SUCCESS'); return True
            self.last_feedback_source=source_time_ms; self.complete_source=max(source_time_ms,self.sample.source_time_ms)
            self.state='WAIT_SETTLE'; self.deadline=now+self.l.settle_ms; self.stable=0; return True
        return False

    def vision_done(self, now, token, stage, success, task_code=None, pitch_ready=False):
        self.tick(now)
        if self.state!='WAIT_VISION' or token!=self.vision_token or stage!=self.stage: return False
        if success is not True: self._fault('VISION_FAILED'); return True
        if not self._still(self.sample): self._fault('VISION_NOT_STOPPED'); return True
        if stage=='QR':
            if not isinstance(task_code,str) or re.fullmatch(r'[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}',task_code) is None:
                self._fault('INVALID_QR'); return True
            a,b,c,d=task_code.split('+')
            if len(set(a))!=3 or set(a)!=set(c) or len(set(c))!=3 or set(b)!=set('123') or set(d)!=set('123'):
                self._fault('INVALID_QR'); return True
            if pitch_ready is not True: return False  # aggregate handshake is not complete yet
            self.task_code=task_code
        self.goal=None; self.state='NEED_REPLAN' if stage=='QR' else 'DONE'; self.deadline=None
        return True

    def snapshot(self):
        return {'mode':'SIM_ONLY','hardware_authorized':False,'state':self.state,'reason':self.reason,
                'stop_pose':asdict(self.stop_pose) if self.stop_pose else None,'stage':self.stage,
                'vision_token':self.vision_token,'settled_count':len(self.settled),
                'map_id':self.map_id,'task_code':self.task_code,'stop_required':self.state=='FAULT'}
