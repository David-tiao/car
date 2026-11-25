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



# Calibration is deliberately separate from map ROI observation: it uses raw
# laser-frame data and measured perpendicular wall distances, never a guessed TF.
def wall_line(frame, angle_deg, half_deg):
    amin, inc = frame["angle_min"], frame["angle_increment"]
    lo, hi = frame["range_min"], frame["range_max"]
    ranges = frame["ranges"]
    if (not all(math.isfinite(v) for v in (amin,inc,lo,hi)) or
            not 0 < abs(inc) <= math.pi or not 0 <= lo < hi or
            not 2 <= len(ranges) <= 10000):
        raise ValueError("invalid scan metadata")
    target, half = math.radians(angle_deg),math.radians(half_deg)
    points = []
    for i,r in enumerate(ranges):
        a = amin+i*inc
        delta = math.atan2(math.sin(a-target),math.cos(a-target))
        if abs(delta) <= half and r is not None and math.isfinite(r) and max(lo,.001)<=r<=hi:
            points.append((r*math.cos(a),r*math.sin(a)))
    if len(points)<12:
        raise ValueError("wall sector has fewer than 12 points")
    x,y,uv,aa,ab,bb = centered(points)
    theta = .5*math.atan2(2*ab,aa-bb)
    c,s = math.cos(theta),math.sin(theta)
    along = [c*u+s*v for u,v in uv]
    rms = math.sqrt(sum((-s*u+c*v)**2 for u,v in uv)/len(uv))
    nx,ny = -s,c
    rho = nx*x+ny*y
    if rho < 0:
        nx,ny,rho = -nx,-ny,-rho
    span = max(along)-min(along)
    if span < .25 or rms > .015 or rho < .10:
        raise ValueError("wall fit rejected: insufficient span, excessive residual or range")
    return math.atan2(ny,nx),rho,rms


def wrap_angle(a):
    return math.atan2(math.sin(a),math.cos(a))


def solve_wall_frame(frame, front_m, left_m, front_deg, left_deg, half_deg):
    f,rho_f,ef = wall_line(frame,front_deg,half_deg)
    l,rho_l,el = wall_line(frame,left_deg,half_deg)
    yf,yl = wrap_angle(-f),wrap_angle(math.pi/2-l)
    if abs(wrap_angle(yf-yl)) > math.radians(2):
        raise ValueError("wall normals not perpendicular within 2 degrees; check wall sectors")
    yaw = wrap_angle(yf+.5*wrap_angle(yl-yf))
    return front_m-rho_f,left_m-rho_l,yaw,max(ef,el)


def calibrate_walls(data, front_m, left_m, front_deg, left_deg, half_deg):
    from statistics import median
    if (not all(math.isfinite(v) for v in (front_m,left_m,front_deg,left_deg,half_deg)) or
            not .15<=front_m<=5 or not .15<=left_m<=5 or not 5<=half_deg<=35 or
            abs(wrap_angle(math.radians(front_deg-left_deg)))<=2*math.radians(half_deg)):
        raise ValueError("invalid measured distances or overlapping/invalid sectors")
    if data.get("format")!="car_lidar_raw_v1" or data.get("frame_id")!="laser_frame":
        raise ValueError("unsupported raw capture format or frame")
    frames = data["frames"]
    if not 10<=len(frames)<=200:
        raise ValueError("require 10..200 raw scan frames")
    stamps = [f["stamp_ns"] for f in frames]
    if any(type(v) is not int or v<=0 for v in stamps) or any(a>=b for a,b in zip(stamps,stamps[1:])):
        raise ValueError("invalid or non-monotonic capture stamps")
    fits = [solve_wall_frame(f,front_m,left_m,front_deg,left_deg,half_deg) for f in frames]
    x,y = median(f[0] for f in fits),median(f[1] for f in fits)
    yaw = wrap_angle(fits[0][2]+median(wrap_angle(f[2]-fits[0][2]) for f in fits))
    spread_m = max(math.hypot(f[0]-x,f[1]-y) for f in fits)
    spread_rad = max(abs(wrap_angle(f[2]-yaw)) for f in fits)
    if spread_m>.010 or spread_rad>math.radians(1) or abs(x)>.30 or abs(y)>.30:
        raise ValueError("inconsistent wall fits or implausible translation; no calibration candidate")
    return dict(status="CALIBRATION_CANDIDATE_ONLY",frames=len(fits),
                x_m=x,y_m=y,yaw_rad=yaw,yaw_deg=math.degrees(yaw),
                max_translation_spread_m=spread_m,max_yaw_spread_rad=spread_rad,
                max_wall_rms_m=max(f[3] for f in fits),
                z_m=None,roll_rad=None,pitch_rad=None,
                requires_independent_validation=True)


def capture_frame(msg):
    n = len(msg.ranges)
    scan_duration(n,msg.time_increment,msg.scan_time)
    amin,inc,lo,hi = msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max
    if (msg.header.frame_id!="laser_frame" or not 2<=n<=10000 or
            not all(math.isfinite(v) for v in (amin,inc,lo,hi)) or
            not 0<abs(inc)<=math.pi or not 0<=lo<hi):
        raise ValueError("invalid frame or scan geometry")
    stamp = int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec)
    if stamp<=0 or not 0<=msg.header.stamp.nanosec<1000000000:
        raise ValueError("invalid stamp")
    ranges = [float(r) if math.isfinite(r) and max(lo,.001)<=r<=hi else None for r in msg.ranges]
    if sum(r is not None for r in ranges)<12:
        raise ValueError("insufficient valid returns")
    return dict(stamp_ns=stamp,angle_min=amin,angle_increment=inc,range_min=lo,
                range_max=hi,time_increment=msg.time_increment,scan_time=msg.scan_time,ranges=ranges)


def collect_raw(path, count):
    from pathlib import Path
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    path = Path(path).expanduser().resolve()
    if path.exists() or not path.parent.is_dir():
        raise ValueError("output exists or parent directory missing; choose a new artifact path")
    if not 10<=count<=200:
        raise ValueError("capture requires 10..200 frames")
    frames = []
    failure = []
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("car_lidar_raw_capture")
        def receive(msg):
            if failure or len(frames)>=count:
                return
            try:
                if node.count_publishers("/scan")!=1:
                    raise ValueError("require exactly one discovered /scan publisher")
                f = capture_frame(msg)
                age = (node.get_clock().now().nanoseconds-f["stamp_ns"])/1e9
                if not -.1<=age<=.6:
                    raise ValueError("stale or future scan stamp")
                if frames and f["stamp_ns"]<=frames[-1]["stamp_ns"]:
                    raise ValueError("non-monotonic scan")
                frames.append(f)
            except (ValueError,OverflowError) as exc:
                failure.append(str(exc))
        subscription = node.create_subscription(LaserScan,"/scan",receive,qos_profile_sensor_data)
        deadline = time.monotonic()+max(15.,count*.5)
        while len(frames)<count and not failure and time.monotonic()<deadline and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
        if failure or len(frames)!=count or node.count_publishers("/scan")!=1:
            raise RuntimeError(f"capture incomplete: {len(frames)}/{count}; "+("; ".join(failure) or "timeout or publisher conflict"))
        # Exclusive create: no overwrite; no output before a complete capture.
        payload = json.dumps(dict(format="car_lidar_raw_v1",frame_id="laser_frame",
                                  physical_stationary_required=True,frames=frames),allow_nan=False)
        if len(payload)>32*1024*1024:
            raise ValueError("capture exceeds 32 MiB")
        with path.open("x",encoding="utf-8") as out:
            out.write(payload+"\n")
        print(f"CAPTURED {len(frames)} frames: {path} (not a calibration result)")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def calibration_self_test():
    # Known synthetic extrinsic, two perpendicular walls at known base distances.
    from types import SimpleNamespace
    x,y,yaw = .045,-.025,.17
    front,left = .80,.65
    frames = []
    n = 720
    inc = 2*math.pi/n
    for j in range(12):
        ranges = []
        for i in range(n):
            a = -math.pi+i*inc+yaw
            u,v = math.cos(a),math.sin(a)
            hits = [(front-x)/u if u>1e-8 else math.inf,
                    (left-y)/v if v>1e-8 else math.inf]
            r = min(hits)
            ranges.append(r if .1<=r<=8 else None)
        frames.append(dict(stamp_ns=1000000000+j*140000000,angle_min=-math.pi,
                           angle_increment=inc,range_min=.1,range_max=8.,ranges=ranges))
    data = dict(format="car_lidar_raw_v1",frame_id="laser_frame",frames=frames)
    result = calibrate_walls(data,front,left,-math.degrees(yaw),90-math.degrees(yaw),20.)
    assert math.dist((result["x_m"],result["y_m"]),(x,y))<1e-8
    assert abs(wrap_angle(result["yaw_rad"]-yaw))<1e-8 and result["z_m"] is None
    for bad in (dict(data,frames=frames[:3]),dict(data,frames=frames[:11]+[frames[0]])):
        try:
            calibrate_walls(bad,front,left,0,90,20)
        except ValueError:
            pass
        else:
            raise AssertionError("bad capture accepted")
    for sectors in ((0,0,20),(0,90,1)):
        try:
            calibrate_walls(data,front,left,*sectors)
        except ValueError:
            pass
        else:
            raise AssertionError("bad sectors accepted")
    stamp = SimpleNamespace(sec=1,nanosec=0)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id="laser_frame",stamp=stamp),
                          angle_min=0.,angle_increment=.01,range_min=.1,range_max=8.,
                          time_increment=.001,scan_time=.1,ranges=[1.]*12+[float("nan"),0.,float("inf")])
    assert capture_frame(msg)["ranges"][-3:]==[None,None,None]
    print("PASS: two-wall planar extrinsic, invalid capture/sector rejection, raw invalid range encoding")


def calibration_cli():
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description="Read-only laser calibration tools; no TF or motor commands")
    commands = parser.add_subparsers(dest="mode",required=True)
    capture = commands.add_parser("capture")
    capture.add_argument("output")
    capture.add_argument("--frames",type=int,default=30)
    solve = commands.add_parser("calibrate")
    solve.add_argument("input")
    solve.add_argument("--front-m",type=float,required=True)
    solve.add_argument("--left-m",type=float,required=True)
    solve.add_argument("--front-deg",type=float,required=True)
    solve.add_argument("--left-deg",type=float,required=True)
    solve.add_argument("--half-deg",type=float,default=20.)
    args = parser.parse_args()
    try:
        if args.mode=="capture":
            collect_raw(args.output,args.frames)
        else:
            path = Path(args.input).expanduser()
            if path.stat().st_size>32*1024*1024:
                raise ValueError("capture exceeds 32 MiB")
            with path.open(encoding="utf-8") as src:
                data = json.load(src)
            print(json.dumps(calibrate_walls(data,args.front_m,args.left_m,args.front_deg,
                                             args.left_deg,args.half_deg),indent=2,allow_nan=False))
    except (ValueError,TypeError,KeyError,OSError,RuntimeError,OverflowError) as exc:
        parser.exit(1,f"[ERROR] {exc}\n")



# Startup mode examines ALL field-facing returns first. The two search bands
# are rule-based classification priors only; no coordinates are assigned without
# laser evidence. A 2-D scan cannot decode QR or see a disk above its scan plane.
FULL_FIELD_ROI = (-.20, -.20, 2.65, 2.72)


def select_roi_runs(full_runs, roi):
    xmin, ymin, xmax, ymax = roi
    selected = []
    for run in full_runs:
        segment = []
        for p in run:
            if xmin <= p[0] <= xmax and ymin <= p[1] <= ymax:
                segment.append(p)
            elif segment:
                selected.append(segment)
                segment = []
        if segment:
            selected.append(segment)
    return selected


def startup_full_field(ranges, angle_min, increment, rmin, rmax, laser_pose, base_pose):
    # Reject partial-FOV frames: "all-field" must not silently use a small arc.
    sweep = max(0, len(ranges)-1)*abs(increment)
    if not math.radians(330) <= sweep <= 2*math.pi+.03:
        raise ValueError("startup requires a near-360-degree LaserScan")
    all_runs = scan_runs(ranges,angle_min,increment,rmin,rmax,laser_pose,FULL_FIELD_ROI)
    # Every angle is considered once, even when a return lies outside the field.
    return dict(
        all_field_points=sum(map(len,all_runs)),
        qr_board=region_result(select_roi_runs(all_runs,QR_ROI),fit_board,base_pose),
        raw_disk=region_result(select_roi_runs(all_runs,RAW_ROI),fit_disk,base_pose))


class StartupAccumulator:
    """One bounded, stationary startup snapshot; never a navigation command."""
    def __init__(self, total=20, min_hits=5):
        if not 5<=total<=200 or not 2<=min_hits<=total:
            raise ValueError("invalid startup scan frame/hit thresholds")
        self.total,self.min_hits = total,min_hits
        self.frames = self.points = self.last_stamp = 0
        self.evidence = {"qr_board":[],"raw_disk":[]}
        self.ambiguous = {"qr_board":0,"raw_disk":0}
        self.finished = False
        self.terminated_by = "FRAME_LIMIT"

    def observe(self, result, stamp_ns):
        if self.finished:
            return
        if stamp_ns <= self.last_stamp:
            raise ValueError("nonmonotonic startup scan stamp")
        self.last_stamp = stamp_ns
        self.frames += 1
        self.points += result["all_field_points"]
        for name in self.evidence:
            entry = result[name]
            if entry["status"] == "AMBIGUOUS":
                self.ambiguous[name] += 1
            elif entry["status"] == "GEOMETRY_CANDIDATE":
                cand = entry["candidates"][0]
                self.evidence[name].append((self.frames,cand["center_map_m"],cand["center_base_m"]))
        if self.frames >= self.total:
            self.finished = True

    def finish(self, reason="TIMEOUT"):
        if not self.finished:
            self.finished,self.terminated_by = True,reason

    def target(self,name):
        from statistics import median
        evidence = self.evidence[name]
        tracks = []
        # Spatially separate different physical candidates: do not take an
        # average across two different board positions or disk-sized objects.
        for frame,p,b in evidence:
            matches = [t for t in tracks if
                       math.dist(p,[median(e[1][0] for e in t),median(e[1][1] for e in t)]) <= .06]
            if matches:
                matches[0].append((frame,p,b))
            else:
                tracks.append([(frame,p,b)])
        confirmed = [t for t in tracks if len(t)>=self.min_hits]
        result = dict(state="UNOBSERVED", observed_frames=len(evidence),
                      ambiguous_frames=self.ambiguous[name],
                      identity_verified=False, position_map_m=None,
                      position_base_m=None)
        if len(confirmed)>1 or self.ambiguous[name]>=self.min_hits:
            result["state"]="AMBIGUOUS"
            return result
        if not confirmed:
            result["state"]="INSUFFICIENT_EVIDENCE" if evidence else "UNOBSERVED"
            return result
        matched = confirmed[0]
        x,y = median(e[1][0] for e in matched),median(e[1][1] for e in matched)
        spread = max(math.dist(e[1],[x,y]) for e in matched)
        if spread>.04:
            result.update(state="UNSTABLE",max_spread_m=spread)
            return result
        bx,by = median(e[2][0] for e in matched),median(e[2][1] for e in matched)
        result.update(state="GEOMETRY_CANDIDATE",
                      position_map_m=[x,y],position_base_m=[bx,by],
                      matched_frames=len(matched),max_spread_m=spread)
        return result

    def snapshot(self):
        qr,disk = self.target("qr_board"),self.target("raw_disk")
        both = all(t["state"]=="GEOMETRY_CANDIDATE" for t in (qr,disk))
        return dict(phase="SNAPSHOT" if self.finished else "SCANNING",
                    status="BOTH_GEOMETRIES_FOUND" if both else "INCOMPLETE",
                    frame_count=self.frames,frame_target=self.total,
                    all_field_points=self.points,scan_stamp_ns=self.last_stamp,
                    terminated_by=self.terminated_by if self.finished else None,
                    map_frame="map",identity_verified=False,
                    navigation_authorized=False,qr_board=qr,raw_disk=disk)


def startup_self_test():
    p = (0,0,0)
    def example(name,cx,cy):
        return {name:dict(status="GEOMETRY_CANDIDATE",
                          candidates=[dict(center_map_m=[cx,cy],
                                           center_base_m=[cx-p[0],cy-p[1]])])}
    good = StartupAccumulator(8,4)
    for n in range(1,9):
        result = dict(all_field_points=300)
        result.update(example("qr_board",2.4,1.17+.001*(n%2)))
        result.update(example("raw_disk",1.2,2.47+.001*(n%2)))
        good.observe(result,n)
    out=good.snapshot()
    assert out["status"]=="BOTH_GEOMETRIES_FOUND"
    assert out["qr_board"]["position_map_m"][0]==2.4
    assert out["raw_disk"]["position_map_m"][0]==1.2
    assert not out["navigation_authorized"] and not out["identity_verified"]
    assert not out["qr_board"]["identity_verified"]
    # No ranges or a partial field of view must never manufacture a position.
    assert startup_full_field([float("nan")]*360,-math.pi,2*math.pi/360,.1,8.,p,p)["qr_board"]["status"]=="NO_RETURN"
    try:
        startup_full_field([1.]*90,0.,math.pi/180,.1,8.,p,p)
    except ValueError:
        pass
    else:
        raise AssertionError("partial field of view accepted")
    # Two persistent, disjoint locations must remain ambiguous.
    clash=StartupAccumulator(10,4)
    for n in range(1,11):
        r=dict(all_field_points=100,raw_disk=dict(status="NO_RETURN"))
        r.update(example("qr_board",2.4,1.05 if n<=5 else 1.35))
        clash.observe(r,n)
    assert clash.snapshot()["qr_board"]["state"]=="AMBIGUOUS"
    missed=StartupAccumulator(5,2)
    for n in range(1,6):
        missed.observe(dict(all_field_points=0,qr_board=dict(status="NO_RETURN"),
                            raw_disk=dict(status="NO_RETURN")),n)
    assert missed.snapshot()["status"]=="INCOMPLETE"
    assert missed.snapshot()["raw_disk"]["position_map_m"] is None
    # A full field scan still isolates the A4 plate from unrelated returns.
    line=[(2.4,1.07+i*.02) for i in range(14)]
    other=[(1.,1.+i*.02) for i in range(10)]
    assert len(select_roi_runs([line,other],QR_ROI))==1
    assert fit_board(select_roi_runs([line,other],QR_ROI)[0]) is not None
    print("PASS: startup full-field FOV, stable coordinate snapshot, ambiguity and missing-target guards")


def main():
    if sys.argv[1:] == ["--self-test"]:
        self_test()
        calibration_self_test()
        startup_self_test()
        return
    if len(sys.argv)>1 and sys.argv[1] in ('capture','calibrate'):
        calibration_cli()
        return
    startup_mode = len(sys.argv)==2 and sys.argv[1]=="startup"
    if len(sys.argv)>1 and not startup_mode:
        raise SystemExit("Use --self-test, capture, calibrate, startup or no arguments")
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
            self.startup = None
            self.startup_pub = None
            self.startup_snapshot = None
            self.startup_first_pose = None
            self.startup_last_pub = 0.
            if startup_mode:
                frames = int(os.environ.get("CAR_STARTUP_FRAMES","20"))
                hits = int(os.environ.get("CAR_STARTUP_HITS","5"))
                self.startup = StartupAccumulator(frames,hits)
                latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE)
                self.startup_pub = self.create_publisher(
                    String,"/competition/lidar/startup_targets",latched)
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
            if self.startup is not None and self.startup.finished:
                return
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

        def publish_startup(self):
            if self.startup_snapshot is None:
                self.startup_snapshot=self.startup.snapshot()
                self.get_logger().warning("STARTUP_SNAPSHOT status="+self.startup_snapshot["status"])
            self.startup_pub.publish(String(data=json.dumps(self.startup_snapshot,allow_nan=False)))
            self.startup_last_pub=time.monotonic()

        def tick(self):
            now = time.monotonic()
            if self.startup is not None:
                if self.startup.finished:
                    self.queue.clear()
                    if now-self.startup_last_pub>=1:
                        self.publish_startup()
                    return
                if now-self.start_wall>=18:
                    self.startup.finish("TIMEOUT")
                    self.publish_startup()
                    return
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
                    if self.startup is not None:
                        if self.startup_first_pose is None:
                            self.startup_first_pose = bp
                        elif not stationary_during_scan(self.startup_first_pose,bp):
                            self.startup.finish("MOTION_DURING_STARTUP")
                            self.publish_startup()
                            continue
                        field = startup_full_field(msg.ranges,msg.angle_min,msg.angle_increment,
                                                   msg.range_min,msg.range_max,lp,bp)
                        self.startup.observe(field,stamp.nanoseconds)
                        if self.startup.finished:
                            self.publish_startup()
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
