"""Read-only competition approach planner and simulated visual handoff.
No ROS, serial, motor or hardware imports. All positions in metres, angles rad.
"""
import copy
import heapq
import math
from collections import defaultdict


def num(v):
    return type(v) in (int, float) and math.isfinite(v)


def vector(v, n):
    return isinstance(v, (tuple, list)) and len(v) == n and all(num(x) for x in v)


def segdist(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    den = dx*dx + dy*dy
    f = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return math.dist(p, (a[0]+f*dx, a[1]+f*dy))


def rectdist(p, r):
    return math.hypot(max(r[0]-p[0], 0., p[0]-r[2]), max(r[1]-p[1], 0., p[1]-r[3]))


def valid_scene(scene, edges):
    if not isinstance(scene, dict) or scene.get('schema') != 'car_startup_scene_v1':
        raise ValueError('INVALID_SCENE')
    if scene.get('status') != 'PREVIEW_READY_UNVERIFIED_TARGETS' or scene.get('ready') is not True:
        raise ValueError('SCENE_NOT_READY')
    if scene.get('identity_verified') is not False or scene.get('navigation_authorized') is not False:
        raise ValueError('UNEXPECTED_AUTHORITY')
    stamp = scene.get('scan_stamp_ns')
    if type(stamp) is not int or stamp <= 0:
        raise ValueError('INVALID_SCAN_STAMP')
    for name in ('qr_board', 'raw_disk'):
        t = scene.get(name)
        if (not isinstance(t, dict) or t.get('state') != 'GEOMETRY_CANDIDATE' or
                t.get('identity_verified') is not False or not vector(t.get('position_map_m'), 2)):
            raise ValueError('INVALID_TARGET')
        x, y = t['position_map_m']
        if not (-.2 <= x <= 2.65 and -.2 <= y <= 2.72):
            raise ValueError('TARGET_OUTSIDE_SCAN_FIELD')
    blocked = scene.get('blocked_roads')
    if not isinstance(blocked, list) or any(type(k) is not str or k not in edges for k in blocked):
        raise ValueError('INVALID_BLOCKED_ROADS')
    obs = scene.get('obstacles')
    if not isinstance(obs, list) or not obs or any(not isinstance(o, dict) or
            not vector([o.get('x'), o.get('y')], 2) or
            not all(0 <= o[k] <= 2.4 for k in ('x', 'y')) for o in obs):
        raise ValueError('INVALID_OBSTACLES')
    return stamp


class Planner:
    ORTHO_YAWS = (0., math.pi/2, math.pi, -math.pi/2)

    def __init__(self, nodes, edges, zones, envelope_m, qr_range_m, disk_range_m,
                 obstacle_uncertainty_m, target_uncertainty_m, sample_m=.04,
                 axis_tol_m=.03, turn_penalty_m=.08):
        # Envelope = circumscribed radius incl. positional error/margin. No assumed real size.
        if (not num(envelope_m) or not .05 <= envelope_m <= .5 or
                not num(obstacle_uncertainty_m) or not 0 <= obstacle_uncertainty_m <= .3 or
                not num(target_uncertainty_m) or not 0 <= target_uncertainty_m <= .3 or
                not num(sample_m) or not .01 <= sample_m <= .06 or
                not num(axis_tol_m) or not 0 < axis_tol_m <= .08 or
                not num(turn_penalty_m) or not 0 <= turn_penalty_m <= .5):
            raise ValueError('invalid footprint/uncertainty')
        for band in (qr_range_m, disk_range_m):
            if not vector(band, 2) or not 0 < band[0] < band[1] <= 1.5:
                raise ValueError('invalid measured observation range')
        if not vector(zones.get('field_size'), 2) or any(abs(x-2.4)>1e-6 for x in zones['field_size']):
            raise ValueError('unexpected field geometry')
        self.rects = list(zones['yellow']) + list(zones['work'])
        if not self.rects or any(not vector(r, 4) or not 0 <= r[0] < r[2] <= 2.4 or
                                  not 0 <= r[1] < r[3] <= 2.4 for r in self.rects):
            raise ValueError('invalid keepout')
        disk = zones['raw_disk']
        if not vector(disk['center'], 2) or not num(disk['radius']) or disk['radius'] <= 0:
            raise ValueError('invalid nominal disk')
        if not nodes or not edges or any(not vector(p, 2) for p in nodes.values()) or any(
                not isinstance(pair, (list, tuple)) or len(pair)!=2 or pair[0]==pair[1] or
                any(n not in nodes for n in pair) for pair in edges.values()):
            raise ValueError('invalid road graph')
        if any(not self.axis_segment(nodes[a], nodes[b]) for a,b in edges.values()):
            raise ValueError('NON_ORTHOGONAL_ROAD_GRAPH')
        self.nodes, self.edges, self.disk = nodes, edges, disk
        self.radius, self.ou, self.tu, self.step = envelope_m, obstacle_uncertainty_m, target_uncertainty_m, sample_m
        self.axis_tol, self.turn_penalty = axis_tol_m, turn_penalty_m
        self.bands = {'qr_board': qr_range_m, 'raw_disk': disk_range_m}

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def axis_segment(a, b):
        return abs(a[0]-b[0]) <= 1e-8 or abs(a[1]-b[1]) <= 1e-8

    def snap_heading(self, yaw):
        closest = min(self.ORTHO_YAWS, key=lambda a: abs(self.wrap(yaw-a)))
        if abs(self.wrap(yaw-closest)) > math.radians(3):
            raise ValueError('START_HEADING_NOT_ORTHOGONAL')
        return closest

    def local_clear(self, a, b, scene):
        # Local connectors and visual corrections must not skirt a locked road.
        if not self.axis_segment(a,b) or not self.segment_clear(a,b,self.circles(scene)):
            return False
        n=max(1,math.ceil(math.dist(a,b)/.02))
        pad=math.dist(a,b)/(2*n)+1e-8
        for k in scene['blocked_roads']:
            u,v=self.edges[k]
            for i in range(n+1):
                p=tuple(a[j]+(b[j]-a[j])*i/n for j in (0,1))
                if segdist(p,self.nodes[u],self.nodes[v]) <= self.radius+.01+pad:
                    return False
        return True

    def turn_cost(self, from_yaw, to_yaw):
        diff = abs(self.wrap(to_yaw - from_yaw))
        quarter_turns = round(diff / (math.pi / 2))
        if abs(diff - quarter_turns * (math.pi / 2)) > 1e-6:
            raise ValueError('non-orthogonal yaw transition')
        return quarter_turns * self.turn_penalty

    def axis_aligned_goal_yaw(self, pose_xy, target_xy):
        dx, dy = target_xy[0] - pose_xy[0], target_xy[1] - pose_xy[1]
        if abs(dx) <= self.axis_tol and abs(dy) > self.axis_tol:
            return math.pi/2 if dy > 0 else -math.pi/2
        if abs(dy) <= self.axis_tol and abs(dx) > self.axis_tol:
            return 0. if dx > 0 else math.pi
        return None

    def circles(self, scene):
        # QR centre is a line-return centroid, not a confirmed board centre; use conservative extent.
        qr = tuple(scene['qr_board']['position_map_m'])
        disk = tuple(scene['raw_disk']['position_map_m'])
        return [(tuple(o[k] for k in ('x','y')), .025+self.ou, 'obstacle') for o in scene['obstacles']] + [
            (tuple(self.disk['center']), self.disk['radius'], 'nominal_disk'),
            (disk, self.disk['radius']+self.tu, 'raw_disk'),
            (qr, .16+self.tu, 'qr_board')]

    def segment_clear(self, a, b, circles, radius=None, sight_target=None):
        rad = self.radius if radius is None else radius
        # Distance to a closed set is 1-Lipschitz: sample with half-step extra margin.
        n = max(1, math.ceil(math.dist(a,b)/.02))
        pad = math.dist(a,b)/(2*n) + 1e-8
        for i in range(n+1):
            p = tuple(a[j]+(b[j]-a[j])*i/n for j in (0,1))
            if sight_target is None and not all(rad+pad <= v <= 2.4-rad-pad for v in p):
                return False
            if any(rectdist(p,r) <= rad+pad for r in self.rects):
                return False
        for p, r, kind in circles:
            if kind == sight_target or (sight_target == 'raw_disk' and kind == 'nominal_disk'):
                continue
            if segdist(p,a,b) <= r+rad+1e-8:
                return False
        return True

    def plan(self, scene, start_pose, target):
        stamp = valid_scene(scene, self.edges)
        if target not in self.bands or not vector(start_pose,3):
            raise ValueError('INVALID_START_OR_TARGET')
        start_yaw = self.snap_heading(start_pose[2])
        start = tuple(start_pose[:2]); dest = tuple(scene[target]['position_map_m'])
        circles = self.circles(scene)
        out = dict(schema='car_fusion_approach_v1', status='NO_SAFE_OBSERVATION_POSE',
                   scan_stamp_ns=stamp, target=target, target_map_m=dest, frame_id='map',
                   identity_verified=False, navigation_authorized=False, preview_only=True,
                   goal_pose=None, path=[], yaw_source='ORTHOGONAL_TRACK_ALIGNED_VIEW',
                   envelope_m=self.radius, orthogonal_heading_only=True,
                   start_heading_rad=start_yaw, actions=[])
        if not self.segment_clear(start,start,circles):
            out['status']='UNSAFE_START'; return out
        # Subdivide unblocked road edges; only add clearance-checked subsegments.
        points = {k:tuple(p) for k,p in self.nodes.items()}; graph = defaultdict(list)
        blocked = set(scene['blocked_roads'])
        def connect(a,b):
            if not self.axis_segment(points[a],points[b]):
                raise ValueError('DIAGONAL_SEGMENT')
            w=math.dist(points[a],points[b]); graph[a].append((b,w)); graph[b].append((a,w))
        for name,(a,b) in sorted(self.edges.items()):
            if name in blocked: continue
            p,q=points[a],points[b]; n=max(1,math.ceil(math.dist(p,q)/self.step))
            chain=[a]
            for i in range(1,n):
                key=name+':'+str(i); points[key]=tuple(p[j]+(q[j]-p[j])*i/n for j in (0,1)); chain.append(key)
            chain.append(b)
            for u,v in zip(chain,chain[1:]):
                if self.segment_clear(points[u],points[v],circles): connect(u,v)
        # A measured off-centre start may use only short, clearance-checked L connectors.
        # Keep the chosen bend as an actual graph node; never draw a diagonal shortcut.
        points['START']=start
        for k in list(graph):
            p=points[k]
            if abs(p[0]-start[0])+abs(p[1]-start[1]) > .35+1e-9: continue
            if self.axis_segment(start,p):
                if self.local_clear(start,p,scene): connect('START',k)
                continue
            for i,bend in enumerate(((p[0],start[1]),(start[0],p[1]))):
                if self.local_clear(start,bend,scene) and self.local_clear(bend,p,scene):
                    key='CONNECT:'+k+':'+str(i)
                    points[key]=bend;connect('START',key);connect(key,k)
        queue=[(0.,'START')]; best={'START':0.}; prev={}
        while queue:
            d,k=heapq.heappop(queue)
            if d>best[k]+1e-9: continue
            for v,w in graph[k]:
                if d+w<best.get(v,math.inf)-1e-9:
                    best[v]=d+w;prev[v]=k;heapq.heappush(queue,(d+w,v))
        lo,hi=self.bands[target]; mid=(lo+hi)/2
        choices=[]
        for k in best:
            if k.startswith('CONNECT:'): continue
            p=points[k]; dist=math.dist(p,dest)
            yaw = self.axis_aligned_goal_yaw(p, dest)
            if yaw is None:
                continue
            if lo<=dist<=hi and self.segment_clear(p,dest,circles,0.,target):
                choices.append((best[k] + abs(dist-mid) + self.turn_cost(start_yaw, yaw), k, yaw))
        if not choices:
            out['status']='NO_ORTHOGONAL_OBSERVATION_POSE'
            return out
        _,goal,yaw=min(choices); route=[goal]
        while route[-1]!='START': route.append(prev[route[-1]])
        path=[points[k] for k in reversed(route)]
        p=points[goal]
        out.update(status='OBSERVATION_POSE_PREVIEW',goal_pose=[*p,yaw],path=path,
                   distance_to_candidate_m=math.dist(p,dest),path_length_m=best[goal],
                   final_turn_penalty_m=self.turn_cost(start_yaw, yaw),
                   actions=orthogonal_actions(path,start_yaw,yaw))
        return out


def orthogonal_actions(path, start_yaw, goal_yaw):
    """Compress translations, keep heading fixed, then issue explicit 90-degree turns.
    The circular envelope already bounds rotation at each checked endpoint.
    Output is a preview action list, not a controller command.
    """
    compact=[]
    for raw in path:
        p=tuple(raw)
        if compact and math.dist(compact[-1],p)<=1e-8:continue
        if compact and not Planner.axis_segment(compact[-1],p):raise ValueError('DIAGONAL_SEGMENT')
        if len(compact)>=2:
            a,b=compact[-2:]
            if Planner.axis_segment(a,p) and sum((b[j]-a[j])*(p[j]-b[j]) for j in (0,1))>0:
                compact[-1]=p;continue
        compact.append(p)
    actions=[]
    for p in compact[1:]:actions.append(dict(kind='TRANSLATE',pose=[*p,start_yaw]))
    yaw=start_yaw
    quarter=round(Planner.wrap(goal_yaw-yaw)/(math.pi/2))
    direction=1 if quarter>0 else -1
    for _ in range(abs(quarter)):
        yaw=Planner.wrap(yaw+direction*math.pi/2)
        actions.append(dict(kind='TURN_90',pose=[*compact[-1],yaw]))
    return actions


def task_code_valid(code):
    if not isinstance(code,str): return False
    g=code.split('+')
    return len(g)==4 and all(len(x)==3 for x in g) and set(g[0])<=set('123456') and len(set(g[0]))==3 and set(g[2])==set(g[0]) and set(g[1])==set(g[3])==set('123')


def visual_step(error_px, jacobian, gain=.5, max_step_m=.02):
    """Simulation only; J= image error change per metre of body translation."""
    if not vector(error_px,2) or not vector(jacobian,4) or not num(gain) or not 0<gain<=1 or not num(max_step_m) or not 0<max_step_m<=.05:
        raise ValueError('INVALID_VISUAL_CALIBRATION')
    a,b,c,d=jacobian; det=a*d-b*c; scale=max(abs(x) for x in jacobian)
    if scale==0 or abs(det)<1e-3*scale*scale: raise ValueError('SINGULAR_VISUAL_CALIBRATION')
    x=-gain*(d*error_px[0]-b*error_px[1])/det; y=-gain*(-c*error_px[0]+a*error_px[1])/det
    length=math.hypot(x,y)
    if not math.isfinite(length): raise ValueError('INVALID_VISUAL_STEP')
    factor=min(1.,max_step_m/length) if length else 1.
    return (x*factor,y*factor)


class SimHandoff:
    """Explicit TEST event contract, not an adapter for the real V4 serial protocol.
    Caller supplies independent sim navigation/vision observations with one session.
    All outputs are intents for inspection only; no actuation authorization.
    """
    def __init__(self, plan, session, jacobian, timeout_s=15., pixel_tol=5., max_steps=8):
        if plan.get('status')!='OBSERVATION_POSE_PREVIEW' or type(session) is not int or session<=0 or not num(timeout_s) or timeout_s<=0 or not num(pixel_tol) or pixel_tol<=0 or type(max_steps) is not int or max_steps<1:
            raise ValueError('INVALID_SIM_START')
        self.plan=plan;self.session=session;self.j=jacobian;self.timeout=timeout_s;self.tol=pixel_tol;self.max_steps=max_steps
        self.state='APPROACH';self.last=-1.;self.started=None;self.last_frame=-1;self.hits=0;self.steps=0;self.code=None
        self.identity=False;self.pending=None;self.last_observation=None;self.not_before=None

    def event(self,e,now):
        if self.state in ('ABORT','DONE'): return self.output()
        if not num(now) or now<0 or now<self.last: self.state='ABORT';return self.output()
        self.last=now
        if self.started is None:self.started=now
        if now-self.started>self.timeout or not isinstance(e,dict) or e.get('conflict') is True:
            self.state='ABORT';return self.output()
        if e.get('session')!=self.session: return self.output()
        typ=e.get('type')
        if typ=='STOP':self.state='ABORT';return self.output()
        if self.state=='APPROACH' and typ=='ARRIVAL':
            pose=e.get('pose'); goal=self.plan['goal_pose']
            if vector(pose,3) and e.get('stationary') is True and math.dist(pose[:2],goal[:2])<=.03 and abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))<=.1:
                self.state='CONFIRM';self.started=now;self.not_before=now
        elif self.state=='CONFIRM' and typ=='VISION':
            frame=e.get('frame'); stamp=e.get('stamp'); pos=e.get('target_map_m')
            if type(frame) is not int or frame<=self.last_frame or not num(stamp) or not 0<=now-stamp<=.4 or stamp<self.not_before:
                self.hits=0;self.code=None;return self.output()
            if self.last_observation is not None and (stamp<=self.last_observation or stamp-self.last_observation>.45):self.hits=0;self.code=None
            self.last_frame=frame;self.last_observation=stamp
            if e.get('stationary') is not True or e.get('pitch_stable') is not True or e.get('target')!=self.plan['target'] or not vector(pos,2) or math.dist(pos,self.plan['target_map_m'])>.10:
                self.hits=0;self.code=None;return self.output()
            if self.plan['target']=='qr_board':
                code=e.get('code')
                if not task_code_valid(code):self.hits=0;self.code=None;return self.output()
                self.hits=self.hits+1 if self.code==code else 1;self.code=code
            else:
                self.hits=self.hits+1 if e.get('disk_geometry_and_colors') is True else 0
            if self.hits>=3:
                self.identity=True
                self.state='WAIT_QR_ACK' if self.plan['target']=='qr_board' else 'ALIGN'
                self.started=now;self.not_before=now;self.hits=0
        elif self.state=='WAIT_QR_ACK' and typ=='QR_ACK' and e.get('code')==self.code:
            self.state='WAIT_PITCH';self.started=now
        elif self.state=='WAIT_PITCH' and typ=='PITCH_READY' and e.get('view')=='RAW' and e.get('stationary') is True:
            self.state='DONE'
        elif self.state=='ALIGN' and typ=='VISION':
            frame=e.get('frame');stamp=e.get('stamp')
            if type(frame) is not int or frame<=self.last_frame or not num(stamp) or not 0<=now-stamp<=.4 or stamp<self.not_before:
                self.hits=0;return self.output()
            if self.last_observation is not None and (stamp<=self.last_observation or stamp-self.last_observation>.45):self.hits=0
            self.last_frame=frame;self.last_observation=stamp
            err=e.get('error_px'); pos=e.get('target_map_m')
            if e.get('stationary') is not True or e.get('pitch_stable') is not True or e.get('target')!=self.plan['target'] or e.get('disk_geometry_and_colors') is not True or not vector(pos,2) or math.dist(pos,self.plan['target_map_m'])>.10 or not vector(err,2):
                self.state='ABORT';return self.output()
            if max(map(abs,err))<=self.tol:
                self.hits+=1
                if self.hits>=3:self.state='DONE'
            elif self.steps>=self.max_steps:self.state='ABORT'
            else:
                try:self.pending=visual_step(err,self.j)
                except ValueError:self.state='ABORT';return self.output()
                self.hits=0;self.steps+=1;self.state='WAIT_STEP';self.started=now
        elif self.state=='WAIT_STEP' and typ=='STEP_DONE' and e.get('step')==self.steps and e.get('stationary') is True:
            self.pending=None;self.state='ALIGN';self.started=now;self.not_before=now;self.last_observation=None
        return self.output()

    def output(self):
        return dict(source='SIM_TEST_ONLY',state=self.state,session=self.session,
                    sim_identity_confirmed=self.identity and self.state!='ABORT',
                    navigation_authorized=False,hardware_authorized=False,
                    step=self.steps,sim_delta_body_m=self.pending if self.state=='WAIT_STEP' else None)



class SimMission:
    """QR -> ACK -> DONE,QR -> IDLE -> raw disk -> visual alignment.
    Explicit synthetic event contract only. No ROS publisher or hardware adapter.
    Every action requires fresh, matching feedback. Abort is latched until a new instance.
    """
    def __init__(self, planner, scene, start_pose, session, jacobian,
                 timeout_s=15., mission_timeout_s=120., max_excursion_m=.08):
        if (type(session) is not int or not 1<=session<65535 or not vector(start_pose,3)
                or not num(timeout_s) or not 0<timeout_s<=60
                or not num(mission_timeout_s) or not timeout_s<=mission_timeout_s<=600
                or not num(max_excursion_m) or not 0<max_excursion_m<=.15):
            raise ValueError('INVALID_MISSION_CONFIG')
        visual_step((0,0),jacobian)  # reject singular calibration before planning
        self.planner=planner; self.scene=copy.deepcopy(scene); self.pose=tuple(start_pose)
        self.session=session; self.j=tuple(jacobian);self.timeout=timeout_s; self.total_timeout=mission_timeout_s
        self.excursion=max_excursion_m; self.last=-1.; self.started=None; self.phase_at=None
        self.last_seq=-1; self.reason=None; self.code=None; self.target='qr_board';self.plans=[]
        self.child=None; self.action=0; self.pending=None; self.phase='PLAN'; self.qr_done=False
        self._plan()

    def _abort(self, reason):
        self.phase='ABORT';self.reason=reason;self.pending=None

    def _plan(self):
        try: self.plan=self.planner.plan(self.scene,self.pose,self.target)
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self._abort('PLAN_REJECTED:'+str(exc));return
        self.plans.append(copy.deepcopy(self.plan))
        if self.plan['status']!='OBSERVATION_POSE_PREVIEW':
            self._abort(self.plan['status']);return
        self.action=0;self.child=SimHandoff(self.plan,self.session,self.j,timeout_s=self.timeout)
        self.phase='NAV';self.pending=None;self.phase_at=self.last if self.last>=0 else None
        # Even a zero-action leg requires a new independent arrival observation.

    @staticmethod
    def _pose_ok(actual, expected, xy_tol=.015):
        return (vector(actual,3) and math.dist(actual[:2],expected[:2])<=xy_tol and
                abs(Planner.wrap(actual[2]-expected[2]))<=math.radians(3))

    def _stationary(self,e,expected,xy_tol=.015):
        return e.get('stationary') is True and self._pose_ok(e.get('pose'),expected,xy_tol)

    def _update_phase(self,phase,now):
        if self.phase!=phase:self.phase=phase;self.phase_at=now

    def _visual(self,e,now):
        if not self._stationary(e,self.pose,xy_tol=.008):
            self._abort('NOT_STATIONARY_OR_POSE_DRIFT');return
        if e.get('view')!=('QR' if self.target=='qr_board' else 'RAW'):
            self._abort('WRONG_CAMERA_VIEW');return
        # Preserve the original temporal / identity gate and reject stale frames there.
        self.child.event(e,now)
        if self.child.state=='ABORT':self._abort('VISION_REJECTED');return
        if self.child.state=='WAIT_QR_ACK':
            self.code=self.child.code;self._update_phase('WAIT_QR_ACK',now)
        elif self.child.state=='ALIGN':self._update_phase('RAW_ALIGN',now)
        elif self.child.state=='DONE':self._update_phase('READY_PICK',now)
        elif self.child.state=='WAIT_STEP':
            dx,dy=self.child.pending
            # Single body axis per step: no diagonal translation and no yaw correction.
            dx,dy=(dx,0.) if abs(dx)>=abs(dy) else (0.,dy)
            yaw=self.plan['goal_pose'][2]
            mx=round(math.cos(yaw))*dx-round(math.sin(yaw))*dy
            my=round(math.sin(yaw))*dx+round(math.cos(yaw))*dy
            goal=(self.pose[0]+mx,self.pose[1]+my,yaw)
            if (math.dist(goal[:2],self.plan['goal_pose'][:2])>self.excursion or
                    not self.planner.local_clear(self.pose[:2],goal[:2],self.scene)):
                self._abort('UNSAFE_VISUAL_STEP');return
            self.pending=dict(kind='MICRO_TRANSLATE',pose=list(goal),step=self.child.steps,delta_body_m=[dx,dy])
            self._update_phase('WAIT_STEP',now)

    def event(self,e,now):
        if self.phase in ('ABORT','READY_PICK'):return self.output()
        if not num(now) or now<0 or now<self.last:
            self._abort('INVALID_CLOCK');return self.output()
        self.last=now
        if self.started is None:self.started=now
        if self.phase_at is None:self.phase_at=now
        if now-self.started>self.total_timeout or now-self.phase_at>self.timeout:
            self._abort('TIMEOUT');return self.output()
        if not isinstance(e,dict) or e.get('source')!='SIM_TEST_ONLY':
            self._abort('INVALID_EVENT_SOURCE');return self.output()
        if e.get('conflict') is True or e.get('type') in ('STOP','SCENE_INVALID','LINK_LOST'):
            self._abort('STOP_OR_CONFLICT');return self.output()
        if e.get('type')=='TICK':return self.output()
        if type(e.get('session')) is not int or e['session']!=self.session:return self.output()
        seq=e.get('seq');stamp=e.get('stamp')
        if (type(seq) is not int or seq<=self.last_seq or not num(stamp)
                or not 0<=now-stamp<=.4 or stamp<self.phase_at):return self.output()
        self.last_seq=seq
        typ=e.get('type')
        if self.phase=='NAV':
            actions=self.plan['actions']
            if self.action<len(actions):
                if typ=='ACTION_DONE' and type(e.get('action')) is int and e['action']==self.action:
                    if not self._stationary(e,actions[self.action]['pose']):self._abort('BAD_NAV_FEEDBACK')
                    else:
                        self.pose=tuple(e['pose']);self.action+=1;self.phase_at=now
            elif typ=='ARRIVAL':
                if not self._stationary(e,self.plan['goal_pose']):self._abort('BAD_ARRIVAL')
                else:
                    self.pose=tuple(e['pose']);self.child.event(e,now)
                    if self.child.state!='CONFIRM':self._abort('ARRIVAL_REJECTED')
                    else:self._update_phase('QR_CONFIRM' if self.target=='qr_board' else 'RAW_CONFIRM',now)
        elif self.phase in ('QR_CONFIRM','RAW_CONFIRM','RAW_ALIGN') and typ=='VISION':self._visual(e,now)
        elif self.phase=='WAIT_QR_ACK' and typ=='QR_ACK' and e.get('code')==self.code:
            if not self._stationary(e,self.pose):self._abort('QR_ACK_WHILE_MOVING')
            else:
                self.child.event(e,now);self._update_phase('WAIT_QR_DONE',now)
        elif self.phase=='WAIT_QR_DONE' and typ=='QR_DONE':
            if not self._stationary(e,self.pose):self._abort('PITCH_WHILE_MOVING')
            else:self._update_phase('WAIT_QR_IDLE',now)
        elif self.phase=='WAIT_QR_IDLE' and typ=='CAMERA_IDLE':
            if not self._stationary(e,self.pose) or e.get('view')!='RAW' or e.get('pitch_stable') is not True:
                self._abort('RAW_VIEW_NOT_READY')
            else:
                self.child.event(dict(e,type='PITCH_READY'),now)
                if self.child.state!='DONE':self._abort('QR_HANDOFF_REJECTED')
                else:
                    self.qr_done=True;self.pose=tuple(e['pose']);self.target='raw_disk'
                    self.session+=1;self.last_seq=-1;self._plan()
        elif self.phase=='WAIT_STEP' and typ=='STEP_DONE' and type(e.get('step')) is int and e['step']==self.pending['step']:
            if not self._stationary(e,self.pending['pose'],xy_tol=.008):self._abort('BAD_MICRO_FEEDBACK')
            elif (math.dist(e['pose'][:2],self.plan['goal_pose'][:2])>self.excursion or
                  not self.planner.segment_clear(e['pose'][:2],e['pose'][:2],self.planner.circles(self.scene))):
                self._abort('UNSAFE_MICRO_FEEDBACK')
            else:
                self.pose=tuple(e['pose']);self.pending=None;self.child.event(e,now)
                if self.child.state!='ALIGN':self._abort('MICRO_ACK_REJECTED')
                else:self._update_phase('RAW_ALIGN',now)
        return self.output()

    def output(self):
        intent=None
        if self.phase=='NAV':
            if self.action<len(self.plan['actions']):
                intent=dict(self.plan['actions'][self.action],action=self.action)
            else:intent=dict(kind='VERIFY_ARRIVAL',pose=self.plan['goal_pose'])
        elif self.phase=='WAIT_STEP':intent=self.pending
        return copy.deepcopy(dict(source='SIM_TEST_ONLY',phase=self.phase,reason=self.reason,
                    session=self.session,target=self.target,pose=self.pose,task_code=self.code,
                    qr_handoff_complete=self.qr_done,sim_ready_pick=self.phase=='READY_PICK',
                    navigation_authorized=False,hardware_authorized=False,
                    identity_verified=False,sim_intent=intent))

def project_pixel_to_plane(pixel, intrinsics, camera_origin_map, rotation_map_camera, plane_point, plane_normal, max_range_m=3.):
    """Calibrated pinhole ray/plane intersection, for undistorted pixel coordinates only.
    R maps optical-frame vectors (+x right,+y down,+z forward) to map.
    Plane is independently calibrated (e.g. disk top or known board plane).
    It must NOT be invented from the candidate centre alone for identity verification.
    Returns a 3-D point in map or raises ValueError; no camera hardware involved.
    """
    if not (vector(pixel,2) and vector(intrinsics,4) and vector(camera_origin_map,3)
            and vector(rotation_map_camera,9) and vector(plane_point,3) and vector(plane_normal,3)
            and num(max_range_m) and 0<max_range_m<=10):
        raise ValueError('INVALID_CAMERA_CALIBRATION')
    fx,fy,cx,cy=intrinsics
    if fx<=0 or fy<=0 or not 0<=pixel[0]<320 or not 0<=pixel[1]<240:
        raise ValueError('INVALID_PIXEL_OR_INTRINSICS')
    rows=[rotation_map_camera[i:i+3] for i in (0,3,6)]
    dot=lambda a,b:sum(x*y for x,y in zip(a,b))
    if any(abs(dot(rows[i],rows[j])-(1 if i==j else 0))>1e-3 for i in range(3) for j in range(3)):
        raise ValueError('INVALID_CAMERA_ROTATION')
    a,b,c=rows
    det=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    if abs(det-1)>1e-3:raise ValueError('INVALID_CAMERA_ROTATION')
    ray=((pixel[0]-cx)/fx,(pixel[1]-cy)/fy,1.)
    ray=[dot(row,ray) for row in rows]; length=math.sqrt(dot(ray,ray));ray=[v/length for v in ray]
    normal_len=math.sqrt(dot(plane_normal,plane_normal))
    if normal_len<=1e-9:raise ValueError('INVALID_PLANE')
    normal=[v/normal_len for v in plane_normal];den=dot(normal,ray)
    if abs(den)<.05:raise ValueError('GRAZING_RAY')
    dist=dot(normal,[a-b for a,b in zip(plane_point,camera_origin_map)])/den
    if not math.isfinite(dist) or not 0<dist<=max_range_m:raise ValueError('INTERSECTION_NOT_VISIBLE')
    result=tuple(a+dist*b for a,b in zip(camera_origin_map,ray))
    if not vector(result,3):raise ValueError('INVALID_PROJECTION')
    return result
