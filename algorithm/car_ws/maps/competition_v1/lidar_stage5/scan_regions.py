#!/usr/bin/env python3
"""Read-only ROS 2 Humble /scan region observer for the competition field.

Run after the existing lidar.sh start and measured map->base_link->laser_frame
TF are available. No motor commands, no TF publication, no obstacle-lock changes.
The ROI bounds are search priors, not measured target positions.
A geometric candidate is NOT a verified QR board or material disk.
Run --self-test without ROS to exercise the geometry.
"""
import json
import math
import os
import sys
import time
from collections import deque

# Map origin lower-left; x right, y up. Bounds in metres (xmin,ymin,xmax,ymax).
# QR center y=1.1..1.3, A4 landscape width .297; raw center x=1.1..1.3, r=.15.
# Figure 1: disk protrudes 75..85 mm into field; with r=.15, center y=2.465..2.475.
# Retain the legacy map's nominal y=2.4 as a search prior, not as a measurement.
# Include the complete possible disk envelope (max y=2.625) plus margin.
QR_ROI = (2.25, 0.90, 2.55, 1.50)
RAW_ROI = (0.90, 2.15, 1.50, 2.65)
MAX_SCAN_TRANSLATION_M = .005  # provisional stationary-test threshold
MAX_SCAN_ROTATION_RAD = math.radians(.3)


def scan_duration(count, time_increment, scan_time):
    """LaserScan stamp is the first ray; conservative end when timing is incomplete."""
    if count < 2 or not all(math.isfinite(v) and v >= 0 for v in (time_increment, scan_time)):
        raise ValueError("invalid scan timing")
    duration = max((count-1)*time_increment, scan_time)
    if not 0 < duration <= .5:
        raise ValueError("missing or implausible scan timing")
    return duration


def stationary_during_scan(first, last):
    yaw = last[2]-first[2]
    return (math.dist(first[:2],last[:2]) <= MAX_SCAN_TRANSLATION_M and
            abs(math.atan2(math.sin(yaw),math.cos(yaw))) <= MAX_SCAN_ROTATION_RAD)


def planar_pose(transform):
    t, q = transform.translation, transform.rotation
    vals = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("nonfinite TF")
    if abs(sum(v*v for v in vals[3:])-1) > .002:
        raise ValueError("nonunit TF")
    # Current geometry assumes a nearly horizontal scan and planar base.
    if abs(q.x) > .01 or abs(q.y) > .01:
        raise ValueError("nonplanar TF")
    return t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y),
                                1-2*(q.y*q.y+q.z*q.z))


def to_map(p, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return (x+c*p[0]-s*p[1], y+s*p[0]+c*p[1])


def to_body(p, pose):
    x, y, a = pose
    dx, dy = p[0]-x, p[1]-y
    c, s = math.cos(a), math.sin(a)
    return (c*dx+s*dy, -s*dx+c*dy)


def scan_runs(ranges, angle_min, increment, rmin, rmax, pose, roi):
    """Keep only actual finite returns; split at invalid samples and gaps."""
    if not all(math.isfinite(v) for v in (angle_min, increment, rmin, rmax)):
        raise ValueError("invalid scan metadata")
    if not 0 < abs(increment) <= math.pi or rmin < 0 or rmax <= rmin:
        raise ValueError("invalid scan metadata")
    xmin, ymin, xmax, ymax = roi
    runs, run = [], []
    for i, r in enumerate(ranges):
        p = None
        if math.isfinite(r) and max(rmin, .001) <= r <= rmax:
            a = angle_min+i*increment
            candidate = to_map((r*math.cos(a), r*math.sin(a)), pose)
            if xmin <= candidate[0] <= xmax and ymin <= candidate[1] <= ymax:
                p = candidate
        if p is None or (run and math.dist(run[-1], p) > .075):
            if run:
                runs.append(run)
                run = []
        if p is not None:
            run.append(p)
    if run:
        runs.append(run)
    # Rejoin scan seam only for a full-circle scan and adjacent valid endpoints.
    step = abs(increment)
    full = abs(len(ranges)*step-2*math.pi) <= 2*step
    if full and len(runs) > 1 and ranges:
        def endpoint(i):
            r = ranges[i]
            if not math.isfinite(r) or not max(rmin,.001) <= r <= rmax:
                return None
            a = angle_min+i*increment
            p = to_map((r*math.cos(a), r*math.sin(a)), pose)
            return p if xmin <= p[0] <= xmax and ymin <= p[1] <= ymax else None
        first, last = endpoint(0), endpoint(len(ranges)-1)
        if first is not None and last is not None and math.dist(first,last) <= .075:
            runs = [runs[-1]+runs[0]]+runs[1:-1]
    return runs


def centered(points):
    n = len(points)
    x = sum(p[0] for p in points)/n
    y = sum(p[1] for p in points)/n
    uv = [(p[0]-x,p[1]-y) for p in points]
    aa = sum(u*u for u,v in uv)
    ab = sum(u*v for u,v in uv)
    bb = sum(v*v for u,v in uv)
    return x,y,uv,aa,ab,bb


def fit_board(points):
    if len(points) < 5:
        return None
    x,y,uv,aa,ab,bb = centered(points)
    theta = .5*math.atan2(2*ab,aa-bb)
    c,s = math.cos(theta),math.sin(theta)
    along = [c*u+s*v for u,v in uv]
    span = max(along)-min(along)
    rms = math.sqrt(sum((-s*u+c*v)**2 for u,v in uv)/len(uv))
    # Board nominal length .297 m; partial returns are allowed.
    # Tangent should be roughly parallel to the right field edge.
    if not (.12 <= span <= .35 and rms <= .015 and abs(c) <= .35):
        return None
    return dict(center_map_m=[x,y], observed_span_m=span, rms_m=rms,
                tangent_yaw_map_rad=theta, point_count=len(points),
                wall_or_board_unresolved=True)


def fit_disk(points):
    if len(points) < 6:
        return None
    x,y,uv,aa,ab,bb = centered(points)
    det = aa*bb-ab*ab
    if aa+bb <= 0 or det <= 1e-4*(aa+bb)**2:
        return None
    # Algebraic least-squares circle in centered coordinates.
    au = .5*sum(u*(u*u+v*v) for u,v in uv)
    av = .5*sum(v*(u*u+v*v) for u,v in uv)
    cx = x+(bb*au-ab*av)/det
    cy = y+(aa*av-ab*au)/det
    distances = [math.hypot(a-cx,b-cy) for a,b in points]
    radius = sum(distances)/len(distances)
    rms = math.sqrt(sum((r-radius)**2 for r in distances)/len(distances))
    angles = sorted(math.atan2(b-cy,a-cx) for a,b in points)
    gaps = [b-a for a,b in zip(angles,angles[1:])]
    gaps.append(angles[0]+2*math.pi-angles[-1])
    coverage = 2*math.pi-max(gaps)
    if not (.12 <= radius <= .18 and rms <= .015 and
            coverage >= math.pi/4 and 1.05 <= cx <= 1.35 and 2.30 <= cy <= 2.50):
        return None
    return dict(center_map_m=[cx,cy], radius_m=radius, rms_m=rms,
                arc_rad=coverage, point_count=len(points))


def region_result(runs, fitter, base_pose):
    points = [p for run in runs for p in run]
    if not points:
        return dict(status="NO_RETURN", point_count=0, candidates=[])
    nearest = min(math.hypot(*to_body(p,base_pose)) for p in points)
    candidates = []
    for run in runs:
        fit = fitter(run)
        if fit is None:
            continue
        x,y = to_body(fit["center_map_m"],base_pose)
        fit.update(identity_verified=False, center_base_m=[x,y], center_distance_m=math.hypot(x,y),
                   bearing_rad=math.atan2(y,x),
                   nearest_candidate_return_m=min(math.hypot(*to_body(p,base_pose)) for p in run))
        if "tangent_yaw_map_rad" in fit:
            a = fit["tangent_yaw_map_rad"]-base_pose[2]
            fit["perpendicular_line_distance_m"] = abs(-math.sin(a)*x+math.cos(a)*y)
        candidates.append(fit)
    return dict(status=("GEOMETRY_CANDIDATE" if len(candidates)==1 else
                        "AMBIGUOUS" if candidates else "RETURN_ONLY"),
                point_count=len(points), nearest_region_return_m=nearest,
                candidates=candidates)


def self_test():
    if not __debug__:
        raise RuntimeError("self-test requires assertions; unset PYTHONOPTIMIZE")
    line = [(2.4,1.07+i*.02) for i in range(14)]
    arc = [(1.2+.15*math.cos(a),2.4+.15*math.sin(a))
           for a in [-2.8+i*.15 for i in range(17)]]
    board = fit_board(line)
    disk = fit_disk(arc)
    assert board and abs(board["observed_span_m"]-.26)<1e-8
    assert disk and math.dist(disk["center_map_m"],(1.2,2.4))<1e-8
    assert abs(disk["radius_m"]-.15)<1e-8
    assert fit_disk(line) is None and fit_board(arc) is None
    assert fit_board(line[:2]) is None and fit_disk(arc[:3]) is None
    assert region_result([],fit_board,(0,0,0))["status"]=="NO_RETURN"
    assert region_result([line,line],fit_board,(0,0,0))["status"]=="AMBIGUOUS"
    p = (1.0,.5)
    pose = (.2,.4,1.2)
    assert math.dist(to_body(to_map(p,pose),pose),p)<1e-8
    runs = scan_runs([1.,float("nan"),1.],0.,.01,.1,8.,
                     (0,0,0),(.9,-.1,1.1,.1))
    assert [len(r) for r in runs]==[1,1]
    assert region_result([line],fit_board,(2.,1.2,0))["candidates"][0]["wall_or_board_unresolved"]
    candidate = region_result([line],fit_board,(2.,1.2,0))["candidates"][0]
    assert not candidate["identity_verified"] and abs(candidate["perpendicular_line_distance_m"]-.4)<1e-8
    assert abs(scan_duration(360,.0004,.14)-.1436)<1e-8
    for args in ((1,.01,.1),(360,0,0),(360,-.1,.1),(360,.01,.1)):
        try:
            scan_duration(*args)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid timing accepted")
    assert stationary_during_scan((0,0,math.pi-.001),(0,0,-math.pi+.001))
    assert not stationary_during_scan((0,0,0),(.01,0,0))
    assert not stationary_during_scan((0,0,0),(0,0,.02))
    # Full-circle seam and invalid endpoint, both scan directions.
    for increment in (math.pi/180,-math.pi/180):
        ranges = [float("nan")]*360
        ranges[:4] = [1.]*4
        ranges[-4:] = [1.]*4
        assert [len(r) for r in scan_runs(ranges,0,increment,.1,8.,(0,0,0),(.9,-.2,1.1,.2))]==[8]
        ranges[0] = float("nan")
        assert len(scan_runs(ranges,0,increment,.1,8.,(0,0,0),(.9,-.2,1.1,.2)))==2
    # Ray casting with true first surface intersections, not pre-made fitted points.
    for cx in (1.10,1.20,1.30):
        for cy in (2.4,2.465,2.475):
            for count in (360,1440):
                pose = (cx,1.85,.2)
                ranges = []
                inc = 2*math.pi/count
                for i in range(count):
                    a = -math.pi+i*inc+pose[2]
                    u,v = math.cos(a),math.sin(a)
                    projection = (cx-pose[0])*u+(cy-pose[1])*v
                    disc = projection**2-((cx-pose[0])**2+(cy-pose[1])**2-.15**2)
                    distance = projection-math.sqrt(max(0.,disc))
                    ranges.append(distance if disc >= 0 and distance > 0 else float("nan"))
                result = region_result(scan_runs(ranges,-math.pi,inc,.1,8.,pose,RAW_ROI),fit_disk,pose)
                assert result["status"]=="GEOMETRY_CANDIDATE", (cx,cy,count,result)
                assert math.dist(result["candidates"][0]["center_map_m"],(cx,cy))<1e-6
                assert abs(result["candidates"][0]["nearest_candidate_return_m"]-(cy-pose[1]-.15))<.002
    for cy in (1.10,1.20,1.30):
        pose = (1.85,cy,.15)
        ranges = []
        inc = math.pi/720
        for i in range(1440):
            a = -math.pi+i*inc+pose[2]
            u,v = math.cos(a),math.sin(a)
            d = .55/u if u > 1e-8 else -1
            ranges.append(d if d > 0 and abs(d*v)<=.297/2 else float("nan"))
        result = region_result(scan_runs(ranges,-math.pi,inc,.1,8.,pose,QR_ROI),fit_board,pose)
        assert result["status"]=="GEOMETRY_CANDIDATE"
        assert abs(result["candidates"][0]["perpendicular_line_distance_m"]-.55)<1e-6
    # A long wall should not be accepted as a board; a short coplanar patch is ambiguous.
    assert fit_board([(2.4,.91+i*.01) for i in range(59)]) is None
    assert fit_board(line)["wall_or_board_unresolved"]
    print("PASS: geometry, 21 ray-cast cases (including rule disk edge), seam, timing, motion gate, wall ambiguity")


def main():
    if sys.argv[1:] == ["--self-test"]:
        self_test()
        return
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    class Observer(Node):
        def __init__(self):
            super().__init__("competition_region_scan")
            self.declare_parameter("qr_roi",list(QR_ROI))
            self.declare_parameter("raw_roi",list(RAW_ROI))
            self.rois = [tuple(self.get_parameter(k).value) for k in ("qr_roi","raw_roi")]
            for r in self.rois:
                if len(r)!=4 or not all(math.isfinite(v) for v in r) or not (r[0]<r[2] and r[1]<r[3]):
                    raise ValueError("ROI must be finite xmin,ymin,xmax,ymax")
            self.buf = Buffer()
            self.listener = TransformListener(self.buf,self)
            self.pub = self.create_publisher(String,"/competition/lidar/regions",10)
            self.queue = deque(maxlen=8)
            self.last_rx = time.monotonic()
            self.last_report = -math.inf
            self.last_status = "WAITING_FOR_SCAN"
            self.last_stamp = -1
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.start_wall = time.monotonic()
            self.startup_timeout = float(os.environ.get("CAR_REGION_STARTUP_TIMEOUT","0"))
            if not math.isfinite(self.startup_timeout) or self.startup_timeout < 0:
                raise ValueError("invalid startup timeout")
            self.ready = False
            self.sub = self.create_subscription(LaserScan,"/scan",self.on_scan,qos_profile_sensor_data)
            self.timer = self.create_timer(.05,self.tick,clock=self.steady)

        def report(self,data):
            self.pub.publish(String(data=json.dumps(data,allow_nan=False)))
            self.last_report = time.monotonic()
            self.last_status = data["status"]
            if data["status"] == "OBSERVATION_ONLY" and not self.ready:
                self.ready = True
                self.get_logger().info("READY: valid scan and time-aligned stationary TF; observation only")

        def on_scan(self,msg):
            self.last_rx = time.monotonic()
            if self.count_publishers("/scan") > 1:
                self.queue.clear()
                self.report(dict(status="MULTIPLE_SCAN_PUBLISHERS"))
                return
            try:
                stamp = Time.from_msg(msg.header.stamp)
            except (ValueError, OverflowError) as exc:
                self.queue.clear()
                self.report(dict(status="INVALID_SCAN",reason=str(exc)))
                return
            if stamp.nanoseconds <= 0 or msg.header.frame_id != "laser_frame" or not msg.ranges:
                self.queue.clear()
                self.report(dict(status="INVALID_SCAN"))
                return
            if stamp.nanoseconds <= self.last_stamp:
                self.queue.clear()
                self.report(dict(status="NONMONOTONIC_SCAN"))
                return
            self.last_stamp = stamp.nanoseconds
            try:
                duration = scan_duration(len(msg.ranges),msg.time_increment,msg.scan_time)
                if not (math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and
                        0 <= msg.range_min < msg.range_max and
                        any(math.isfinite(r) and max(.001,msg.range_min)<=r<=msg.range_max for r in msg.ranges)):
                    raise ValueError("no valid ranges or invalid range limits")
            except ValueError as exc:
                self.queue.clear()
                self.report(dict(status="INVALID_SCAN",reason=str(exc)))
                return
            if len(self.queue)==self.queue.maxlen:
                self.queue.clear()
                self.report(dict(status="SCAN_QUEUE_OVERFLOW"))
            self.queue.append((self.last_rx,stamp,msg,duration))

        def tick(self):
            now = time.monotonic()
            if not self.ready and self.startup_timeout and now-self.start_wall > self.startup_timeout:
                raise RuntimeError("region startup timed out: require valid scan and measured stationary TF; last status="+self.last_status)
            if now-self.last_rx > .6:
                self.queue.clear()
                if now-self.last_report > .5:
                    self.report(dict(status="SCAN_STALE"))
                return
            while self.queue:
                received,stamp,msg,duration = self.queue[0]
                age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
                if now-received > .4 or age > .6 or age < -.1:
                    self.queue.popleft()
                    self.report(dict(status="TF_OR_SCAN_STALE",stamp_ns=stamp.nanoseconds))
                    continue
                try:
                    laser = self.buf.lookup_transform("map",msg.header.frame_id,stamp)
                    base = self.buf.lookup_transform("map","base_link",stamp)
                    end = stamp+Duration(seconds=duration)
                    laser_end = self.buf.lookup_transform("map",msg.header.frame_id,end)
                    base_end = self.buf.lookup_transform("map","base_link",end)
                except TransformException:
                    # Never block the listener or use latest-time TF instead.
                    return
                self.queue.popleft()
                try:
                    lp,bp = planar_pose(laser.transform),planar_pose(base.transform)
                    if not (stationary_during_scan(lp,planar_pose(laser_end.transform)) and
                            stationary_during_scan(bp,planar_pose(base_end.transform))):
                        self.report(dict(status="MOTION_UNSUPPORTED",stamp_ns=stamp.nanoseconds))
                        continue
                    results = []
                    for roi,fitter in zip(self.rois,(fit_board,fit_disk)):
                        runs = scan_runs(msg.ranges,msg.angle_min,msg.angle_increment,
                                         msg.range_min,msg.range_max,lp,roi)
                        results.append(region_result(runs,fitter,bp))
                    self.report(dict(status="OBSERVATION_ONLY",stamp_ns=stamp.nanoseconds,
                                     map_frame="map",body_frame="base_link",
                                     identity_verified=False, stationary_test_only=True,
                                     qr_board=results[0],raw_disk=results[1]))
                except (ValueError,OverflowError) as exc:
                    self.report(dict(status="INVALID_GEOMETRY",reason=str(exc)))

    rclpy.init()
    node = None
    try:
        node = Observer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
