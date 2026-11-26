"""ROS-independent, single-threaded Pi motion transport supervisor.
No auto-reconnect, no implicit grant, no map->odom assumption. Caller owns clock.
"""
import json
import math
from pi_session import PiSession
from motion_link import Frame, STOP, split_orthogonal_action

SAFETY_NS = 250_000_000
GOAL_NS = 250_000_000
FEEDBACK_S = .36
KINDS = ('TRANSLATE', 'TURN_90', 'MICRO_TRANSLATE')
CARDINAL = (0, 90000, -180000, -90000)


def integer(v, lo, hi):
    return type(v) is int and lo <= v <= hi


def pose(v, cardinal=False):
    return (isinstance(v, (tuple, list)) and len(v) == 3 and
            all(integer(a, -10000, 10000) for a in v[:2]) and
            integer(v[2], -180000, 179999) and (not cardinal or v[2] in CARDINAL))


def parse(raw):
    if not isinstance(raw, str) or len(raw) > 4096:
        raise ValueError('oversized or invalid JSON')
    def unique(pairs):
        out = {}
        for k, v in pairs:
            if k in out: raise ValueError('duplicate JSON key')
            out[k] = v
        return out
    obj = json.loads(raw, object_pairs_hook=unique,
                     parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    if not isinstance(obj, dict): raise ValueError('expected object')
    return obj


class MotionBridge:
    """Transport is injected (read/write/close); no hardware is opened here.
    Local operator configuration + independent MCU grant + live external permit
    are all necessary. The permit is NOT a collision detector or hardware e-stop.
    """
    def __init__(self, session, transport, *, operator_enabled=False):
        if type(operator_enabled) is not bool: raise ValueError('bad operator flag')
        self.transport = transport
        self.operator_enabled = operator_enabled
        self.pi = PiSession(session, self._clearance)
        self.session = session
        self.permit = None
        self.permit_until = None
        self.safety_seq = 0
        self.goal_seq = 0
        self.active_goal = None
        self.finished_goal = None
        self.last = -1.
        self.last_feedback = None
        self.feedback_seq = 0
        self.closed = False
        self.fault = None
        self.stop_attempted = False
        self.stop_written = False
        self._now = 0.

    def _clock(self, now):
        if type(now) not in (float, int) or not math.isfinite(now) or now < 0 or now < self.last:
            self._fail('INVALID_CLOCK'); return False
        self.last = self._now = now
        return True

    def _fail(self, reason):
        if self.fault is None: self.fault = str(reason)
        # Public PiSession.tick handles revocation and returns one STOP intent.
        if self.pi.phase != 'NEW' and not self.stop_attempted:
            self.stop_attempted = True
            try:
                wire = self.pi.tick(max(0., self._now), False)
                if wire is not None and Frame.decode(wire).kind == STOP:
                    self._write(wire); self.stop_written = True
            except Exception:
                pass  # watchdog is the remaining mechanism; no stop confirmation
        self._close()

    def _close(self):
        if not self.closed:
            self.closed = True
            try: self.transport.close()
            except Exception: pass

    def _write(self, wire):
        if self.transport.write(wire) != len(wire):
            raise OSError('partial serial write')

    def _fresh(self, now):
        return self.permit is not None and self.permit_until is not None and now < self.permit_until

    def _contains(self, p):
        if not pose(p) or self.permit is None: return False
        x0, y0, x1, y1 = self.permit['center_bounds_mm']
        return x0 <= p[0] <= x1 and y0 <= p[1] <= y1

    def _clearance(self, start, end, kind):
        # Rectangle contains both ends, hence axis-aligned center segment. External
        # supervisor must already inflate it by footprint, errors and braking.
        p = self.permit
        g = self.active_goal
        return bool(self._fresh(self._now) and g and p['goal_id'] == g['goal_id'] and
                    p['kind'] == kind == g['kind'] and p['start'] == g['start'] and
                    p['goal'] == g['goal'] and self._contains(start) and self._contains(end) and
                    (kind != 'TURN_90' or p['turn_clear'] is True))

    def _permitted(self, now):
        if not self.operator_enabled or not self._fresh(now): return False
        p = self.permit
        if not (p['allow'] is True and p['local_grant_confirmed'] is True and p['tf_verified'] is True):
            return False
        if self.pi.pose is not None and not self._contains(self.pi.pose): return False
        if self.active_goal:
            g = self.active_goal
            if p['goal_id'] != g['goal_id'] or p['kind'] != g['kind'] or p['start'] != g['start'] or p['goal'] != g['goal']:
                return False
            if not self._contains(g['goal']): return False
            if g['kind'] == 'TURN_90' and p['turn_clear'] is not True: return False
            if self.pi.chunk is not None and not self._contains(self.pi.chunk): return False
        return True

    def safety(self, raw, now, wall_ns):
        if self.closed or not self._clock(now): return False
        try:
            p = parse(raw)
            if p.get('schema') != 'car_motion_safety_v1' or p.get('session') != self.session:
                return False
            if type(p.get('session')) is not int: raise ValueError('session type')
            seq = p.get('seq')
            if not integer(seq, 1, 2**31-1): raise ValueError('sequence')
            # A matching revocation dominates stale/replayed sequence numbers.
            if p.get('allow') is False:
                self._fail('SAFETY_REVOKED'); return True
            if seq <= self.safety_seq: return False
            stamp = p.get('stamp_ns')
            if not integer(stamp, 1, 2**63-1) or not integer(wall_ns, 1, 2**63-1) or not 0 <= wall_ns-stamp < SAFETY_NS:
                raise ValueError('stale safety timestamp')
            if p.get('frame_id') != 'odom' or any(p.get(k) is not True for k in ('allow','local_grant_confirmed','tf_verified')):
                raise ValueError('missing independent grant/TF/permission')
            gid = p.get('goal_id'); b = p.get('center_bounds_mm')
            if not integer(gid, 0, 2**31-1) or not isinstance(b, list) or len(b) != 4 or not all(integer(v,-10000,10000) for v in b) or b[0] > b[2] or b[1] > b[3]:
                raise ValueError('invalid clearance bounds')
            if type(p.get('turn_clear')) is not bool: raise ValueError('turn clearance')
            if gid == 0:
                if p.get('kind') != 'IDLE' or p.get('start') is not None or p.get('goal') is not None:
                    raise ValueError('idle permit')
            else:
                if p.get('kind') not in KINDS or not pose(p.get('start'),True) or not pose(p.get('goal'),True):
                    raise ValueError('invalid action permit')
                split_orthogonal_action(p['start'],p['goal'],p['kind'])
            self.permit = p; self.safety_seq = seq
            self.permit_until = now + (SAFETY_NS-(wall_ns-stamp))/1e9
            if self.pi.phase != 'NEW' and not self._permitted(now):
                self._fail('SAFETY_SCOPE_CHANGED'); return False
            return True
        except (ValueError, TypeError, OverflowError, KeyError, RecursionError):
            self._fail('INVALID_SAFETY'); return False

    def goal(self, raw, now, wall_ns):
        if self.closed or not self._clock(now): return False
        try:
            g = parse(raw)
            if g.get('schema') != 'car_motion_goal_v1' or g.get('session') != self.session: return False
            gid = g.get('goal_id'); stamp = g.get('stamp_ns')
            if type(g.get('session')) is not int or not integer(gid,1,2**31-1): raise ValueError('goal id')
            if gid <= self.goal_seq: return False  # no replay, even after completion
            if not integer(stamp,1,2**63-1) or not integer(wall_ns,1,2**63-1) or not 0 <= wall_ns-stamp < GOAL_NS: return False
            if self.pi.phase != 'IDLE' or self.active_goal or self.last_feedback is None or now-self.last_feedback >= FEEDBACK_S:
                return False
            if g.get('frame_id') != 'odom' or g.get('kind') not in KINDS or not pose(g.get('start'),True) or not pose(g.get('goal'),True):
                raise ValueError('invalid odom goal')
            split_orthogonal_action(g['start'],g['goal'],g['kind'])
            self.active_goal = g
            if not self._permitted(now) or not self._contains(g['start']):
                self._fail('GOAL_NOT_PERMITTED'); return False
            if not self.pi.queue_action(g['kind'],g['start'],g['goal']):
                self._fail(self.pi.reason or 'GOAL_REJECTED'); return False
            self.goal_seq = gid
            return True
        except (ValueError, TypeError, OverflowError, KeyError, RuntimeError, RecursionError):
            self._fail('INVALID_GOAL'); return False

    def tick(self, now):
        if self.closed or not self._clock(now): return
        if self.pi.phase == 'NEW':
            # Only an explicit IDLE permit starts a session. No motion queued at boot.
            if not self._permitted(now) or self.permit['goal_id'] != 0: return
            self.pi.start(True, True)
        if not self._permitted(now): self._fail('SAFETY_EXPIRED_OR_REVOKED'); return
        try:
            data = self.transport.read(4096)
            if not isinstance(data, bytes) or len(data)>4096: raise OSError('invalid transport read')
            if data and self.pi.receive(data, now):
                self.last_feedback = now; self.feedback_seq += 1
            if self.pi.phase == 'FAULT': self._fail(self.pi.reason or 'PEER_FAULT'); return
            if not self._permitted(now): self._fail('FEEDBACK_OUTSIDE_CLEARANCE'); return
            if self.last_feedback is not None and now-self.last_feedback >= FEEDBACK_S:
                self._fail('FEEDBACK_STALE'); return
            wire = self.pi.tick(now, True)
            if self.pi.phase == 'FAULT':
                # PiSession may have emitted STOP in this tick: do not lose it.
                if wire is not None and Frame.decode(wire).kind == STOP:
                    self.stop_attempted = True; self._write(wire); self.stop_written = True
                self._fail(self.pi.reason or 'SESSION_FAULT'); return
            if wire is not None: self._write(wire)
            if self.active_goal and self.pi.phase == 'IDLE' and self.pi.action is None:
                self.finished_goal = self.active_goal['goal_id']; self.active_goal = None
        except (OSError, ValueError, RuntimeError, TypeError, OverflowError):
            self._fail('TRANSPORT_OR_SESSION_ERROR')

    def shutdown(self):
        if not self.closed: self._fail('SHUTDOWN')

    def status(self, now):
        age = None if self.last_feedback is None else max(0., now-self.last_feedback)
        valid = not self.closed and self.fault is None and age is not None and age < FEEDBACK_S and self.pi.pose is not None
        return dict(schema='car_motion_feedback_v1', session=self.session, frame_id='odom',
                    phase='FAULT' if self.fault else self.pi.phase, reason=self.fault,
                    feedback_seq=self.feedback_seq, pose_mm_mdeg=list(self.pi.pose) if valid else None,
                    pose_valid=valid, feedback_age_s=age, mcu_state=self.pi.host.state if valid else None,
                    active_goal_id=self.active_goal['goal_id'] if self.active_goal else None,
                    completed_goal_id=self.finished_goal, completed_count=self.pi.completed,
                    safety_fresh=self._fresh(now) and not self.closed,
                    stop_attempted=self.stop_attempted, stop_written=self.stop_written,
                    physical_stop_confirmed=False, velocity_available=False,
                    navigation_authorized=False)
