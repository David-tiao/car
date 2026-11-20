#!/usr/bin/env python3
"""工创智能搬运仿真器：零依赖 HTTP 服务器和多人共享场景。"""
from __future__ import annotations

import argparse
import base64
from collections import deque
import math
import time
import ipaddress
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
import secrets
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from web_mode_gateway import mode_get, mode_post
from keyboard_feedback_autostart import stop as stop_keyboard_feedback_observer

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "web"
DATA = ROOT / "data"
SHARED_FILE = DATA / "shared_scene.json"
LOCK = threading.RLock()
MAX_BODY = 48 * 1024
BUILD_VERSION = "13.0.0"
TELEMETRY = {"seq": 0, "last_mono": 0., "samples": deque(maxlen=3000)}
CAMERA_NAMES = ("openmv", "pi5")
CAMERA_CONDITION = threading.Condition()
CAMERA_FEEDS = {name: {"jpeg": None, "seq": 0, "last_mono": 0., "fps_times": deque(maxlen=24), "source": ""} for name in CAMERA_NAMES}
CAMERA_TOKEN_FILE = DATA / "camera_token.txt"
MAX_JPEG_BYTES = 2 * 1024 * 1024


# Browser ROS2 display snapshots. Publishers are restricted to loopback; LAN clients are read-only.
ROS_LOCK = threading.RLock()
ROS_STATE = {"seq": 0, "last_mono": 0.0, "snapshot": None}
ROS_GRIDS = {name: {"seq": 0, "last_mono": 0.0, "grid": None}
             for name in ("map", "global_costmap", "local_costmap")}
ROS_JSON_LIMIT = 240 * 1024
ROS_GRID_JSON_LIMIT = 2 * 1024 * 1024


def is_loopback(addr):
    try:
        return ipaddress.ip_address(addr).is_loopback
    except ValueError:
        return False


def read_json_body(handler, limit):
    if handler.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
        raise ValueError("Content-Type must be application/json")
    try:
        size = int(handler.headers.get("Content-Length", "0"))
    except (TypeError, ValueError):
        raise ValueError("Invalid Content-Length") from None
    if not 0 < size <= limit:
        raise ValueError("JSON payload is empty or too large")
    def reject_constant(value):
        raise ValueError("Invalid JSON number: " + value)
    return json.loads(handler.rfile.read(size), parse_constant=reject_constant)


def ros_status():
    with ROS_LOCK:
        age = (time.monotonic() - ROS_STATE["last_mono"]) if ROS_STATE["seq"] else None
        result = dict(ROS_STATE["snapshot"] or {})
        result.update({"online": age is not None and age < 3.0,
                       "age_s": round(age, 2) if age is not None else None,
                       "seq": ROS_STATE["seq"],
                       "grids": {k: {"seq": v["seq"], "frame": (v["grid"] or {}).get("frame", ""),
                                      "width": (v["grid"] or {}).get("width", 0),
                                      "height": (v["grid"] or {}).get("height", 0),
                                      "age_s": round(time.monotonic()-v["last_mono"], 2) if v["seq"] else None}
                                 for k, v in ROS_GRIDS.items()}})
        return result


def validate_grid(obj):
    if not isinstance(obj, dict):
        raise ValueError("Grid must be an object")
    name = obj.get("name")
    if name not in ROS_GRIDS:
        raise ValueError("Unknown grid layer")
    width, height = int(obj.get("width",0)), int(obj.get("height",0))
    resolution = float(obj.get("resolution", 0))
    ox = float(obj.get("origin_x",0)); oy = float(obj.get("origin_y",0)); yaw = float(obj.get("origin_yaw",0))
    if not 1 <= width <= 1200 or not 1 <= height <= 1200 or width*height > 1500000:
        raise ValueError("Grid dimensions outside limits")
    if not all(math.isfinite(x) for x in (resolution,ox,oy,yaw)) or not 0.001 <= resolution <= 5:
        raise ValueError("Invalid grid resolution or origin")
    packed = obj.get("data")
    if not isinstance(packed, str) or len(packed) > 2100000:
        raise ValueError("Invalid occupancy encoding")
    raw = base64.b64decode(packed, validate=True)
    if len(raw) != width*height:
        raise ValueError("Occupancy array length mismatch")
    return name, {"width":width, "height":height, "resolution":resolution,
                  "origin_x":ox,"origin_y":oy,"origin_yaw":yaw,"frame":str(obj.get("frame", "map"))[:80],
                  "data":packed}


def camera_token():
    # Never return or log this token over the web. Only local trusted host reads the file.
    DATA.mkdir(parents=True, exist_ok=True)
    if CAMERA_TOKEN_FILE.exists():
        return CAMERA_TOKEN_FILE.read_text(encoding="ascii").strip()
    key = secrets.token_urlsafe(32)
    try:
        fd = os.open(CAMERA_TOKEN_FILE, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w", encoding="ascii") as out:
            out.write(key + "\n")
    except FileExistsError:
        return CAMERA_TOKEN_FILE.read_text(encoding="ascii").strip()
    return key

CAMERA_TOKEN = camera_token()


def camera_status():
    now = time.monotonic()
    with CAMERA_CONDITION:
        return {name: {"online": bool(feed["jpeg"]) and now - feed["last_mono"] < 3.0,
                       "seq": feed["seq"], "age_s": round(now - feed["last_mono"], 2) if feed["seq"] else None,
                       "fps": round((len(feed["fps_times"]) - 1) / max(.001, feed["fps_times"][-1] - feed["fps_times"][0]), 1) if len(feed["fps_times"]) >= 2 else 0,
                       "source": feed["source"]} for name, feed in CAMERA_FEEDS.items()}



def validate_scene(scene):
    """Validate two obstacles on the screenshot-matching 5x5 (480mm) numbered grid.

    Legacy V8 scenes are migrated on READ to avoid stale 8x8 layout settings.
    New client writes must explicitly advertise scenarioVersion >= 10.
    """
    if not isinstance(scene, dict):
        raise ValueError("scene must be a JSON object")

    # Accept V2 'blockedSlots' from a saved scene, but convert it to two small real obstacles.
    legacy = scene.get("blockedSlots", scene.get("blocked_slots"))
    points = scene.get("obstacles")
    old_centers = ((1.2, 1.75), (1.75, 1.2), (1.2, .65), (.65, 1.2))
    if points is None and isinstance(legacy, list):
        points = [{"x": old_centers[i-1][0], "y": old_centers[i-1][1]}
                  for i in legacy[:2] if type(i) is int and 1 <= i <= 4]
    if points is None:
        points = []
    if not isinstance(points, list) or len(points) > 2:
        raise ValueError("At most two obstacles are allowed")

    yellow = ((.55,.55,1,1), (1.4,.55,1.85,1),
              (.55,1.4,1,1.85),(1.4,1.4,1.85,1.85))
    blue = ((2.1,2.1,2.4,2.4),(2.1,0,2.4,.3))
    work = ((0,.91,.15,1.49),(.91,0,1.49,.15))
    # Placement bounds transcribed from official initial-field Fig.1.
    try:
        raw_x = float(scene.get("rawX", 1.2))
        raw_y = float(scene.get("rawY", 2.4))
        qr_y = float(scene.get("qrY", 1.2))
    except (ValueError, TypeError):
        raise ValueError("Invalid fixture coordinates") from None
    if not (math.isfinite(raw_x) and 1.1 <= raw_x <= 1.3 and math.isfinite(raw_y) and 2.25 <= raw_y <= 2.55 and math.isfinite(qr_y) and 1.1 <= qr_y <= 1.3):
        raise ValueError("Raw X in 1.1..1.3, Raw Y in 2.25..2.55 and QR Y in 1.1..1.3 m")
    try:
        schema = int(scene.get("scenarioVersion", 0))
        reported_step = float(scene.get("gridStep", .48))
    except (ValueError, TypeError):
        raise ValueError("Invalid grid schema") from None
    grid_step = .48  # V10 grid is fixed; old steps must never override the 5x5 layout.
    if schema >= 10 and abs(reported_step-grid_step) > 1e-8:
        raise ValueError("V10 requires fixed 480 mm cells")
    count = 5
    def cell_id(x,y):
        return min(count-1, int(x/grid_step)), min(count-1, int(y/grid_step))
    def valid_obstacle(x, y):
        r=.025
        c, row = cell_id(x,y)
        cell=(c*grid_step,row*grid_step,(c+1)*grid_step,(row+1)*grid_step)
        # A blocked cell may not intersect either blue start/finish region.
        if any(cell[0]<b[2] and cell[2]>b[0] and cell[1]<b[3] and cell[3]>b[1] for b in blue):
            return False
        if not (r<=x<=2.4-r and r<=y<=2.4-r):
            return False
        for x0,y0,x1,y1 in yellow + blue + work:
            dx=max(x0-x,0,x-x1)
            dy=max(y0-y,0,y-y1)
            if dx*dx+dy*dy < r*r:
                return False
        return (x-raw_x)**2+(y-raw_y)**2 >= (.15+r)**2

    obstacles=[]
    for item in points:
        if not isinstance(item, dict):
            raise ValueError("Invalid obstacle")
        try:
            x, y = float(item["x"]), float(item["y"])
        except (ValueError, TypeError, KeyError):
            raise ValueError("Invalid obstacle coordinates") from None
        # Regardless of click position, a grid obstacle occupies the cell centre.
        c, row = cell_id(x, y)
        x, y = round((c+.5)*grid_step, 4), round((row+.5)*grid_step, 4)
        if not valid_obstacle(x,y):
            if schema < 10:
                continue  # Old 8x8 positions which are illegal on 5x5 are dropped.
            raise ValueError("Obstacle cell conflicts with keepout zone or start/finish")
        if any(cell_id(o["x"],o["y"])==cell_id(x,y) for o in obstacles):
            if schema < 10:
                continue
            raise ValueError("Only one obstacle per occupied grid cell")
        obstacles.append({"x":round(x,4),"y":round(y,4)})
    result={"obstacles":obstacles, "scenarioVersion": 10, "rawX":round(raw_x,3), "rawY":round(raw_y,3), "qrY":round(qr_y,3), "gridStep":grid_step}

    for key in ("start", "goal"):
        value = scene.get(key)
        if value is not None:
            if not isinstance(value, dict):
                raise ValueError("Position must be an object")
            try:
                x, y = float(value["x"]), float(value["y"])
            except (KeyError, ValueError, TypeError):
                raise ValueError("Invalid position coordinates") from None
            if not (0 <= x <= 2.4 and 0 <= y <= 2.4):
                raise ValueError("Position outside field")
            result[key] = {"x": round(x, 4), "y": round(y, 4)}

    if scene.get("startZone") in (0, 1, 2):
        result["startZone"] = scene["startZone"]
    old_scenario = int(scene.get("scenarioVersion", 0)) < 8
    for key, minimum, maximum in (("velocity", .05, 1.2), ("handling", .05, 3.), ("safetyMargin", 0, .05),
                                   ("rotorDwell", 0, 15), ("rotorRate", 36, 60), ("rotorAngle", 5, 180),
                                   ("alignOffset", 0, .04), ("alignSpeed", .01, .12), ("alignYaw", 0, 10),
                                   ("armReach", 0, .15), ("armSpeed", .02, .2)):
        if key in scene:
            try:
                value = float(scene[key])
            except (ValueError, TypeError):
                raise ValueError(f"Invalid {key}") from None
            if not minimum <= value <= maximum:
                if old_scenario and key in ("rotorRate", "rotorAngle"):
                    continue  # Migrate previous 30° defaults to new rule-based 120° / 8 s per revolution.
                raise ValueError(f"Invalid {key}")
            if old_scenario and key in ("rotorRate", "rotorAngle"):
                continue
            result[key] = round(value, 3)
    if scene.get("mode") in ("direct", "mission"):
        result["mode"] = scene["mode"]
    if scene.get("planner") in ("astar", "dijkstra", "dstar"):
        result["planner"] = scene["planner"]
    if scene.get("goalPolicy") in ("other", "home"):
        result["goalPolicy"] = scene["goalPolicy"]
    if "task" in scene:
        value = scene["task"]
        if not isinstance(value, str) or len(value) > 40:
            raise ValueError("Invalid task code")
        result["task"] = value
    for key in ("showGrid", "ringFollow"):
        if key in scene and isinstance(scene[key], bool):
            result[key] = scene[key]
    if schema < 10:
        result["showGrid"] = True  # A V8 hidden grid must not hide the new numbered map.
    if scene.get("pathView") in ("current", "all", "global"):
        result["pathView"] = ("global" if scene["pathView"] == "all" else scene["pathView"])
    if scene.get("turnDirection") in ("cw", "ccw"):
        result["turnDirection"] = scene["turnDirection"]
    return result


def get_saved():
    if not SHARED_FILE.exists():
        return {"revision": 0, "scene": None}
    try:
        obj = json.loads(SHARED_FILE.read_text("utf-8"))
        if not isinstance(obj, dict):
            raise ValueError()
        return {"revision": int(obj["revision"]), "scene": validate_scene(obj["scene"])}
    except (ValueError, OSError, TypeError, KeyError):
        print("[警告] 共享场景文件无效，使用空场景。", flush=True)
        return {"revision": 0, "scene": None}


SHARED = get_saved()


def local_ipv4s():
    ips = []
    try:
        proc = subprocess.run(["ip", "-o", "-4", "addr", "show", "scope", "global"], capture_output=True, text=True, timeout=2)
        for line in proc.stdout.splitlines():
            for token in line.split():
                if "/" in token:
                    try:
                        addr = ipaddress.ip_interface(token).ip
                        if addr.is_private and not addr.is_loopback:
                            ips.append(str(addr))
                    except ValueError:
                        pass
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    if not ips:
        try:
            for entry in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                value = entry[4][0]
                if not value.startswith("127.") and value not in ips:
                    ips.append(value)
        except OSError:
            pass
    return sorted(set(ips))


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB), **kwargs)

    def end_headers(self):
        # Avoid stale V1/V2/V3 pages, including 304 responses from browser cache.
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        super().end_headers()

    def respond_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream_camera(self, name):
        # An MJPEG stream is just image/jpeg parts over a persistent HTTP connection.
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=carframe")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        seq = 0
        try:
            while True:
                with CAMERA_CONDITION:
                    CAMERA_CONDITION.wait_for(lambda: CAMERA_FEEDS[name]["seq"] != seq, timeout=3.0)
                    latest = CAMERA_FEEDS[name]
                    if not latest["jpeg"] or latest["seq"] == seq:
                        continue
                    frame = latest["jpeg"]
                    seq = latest["seq"]
                part = (b"--carframe\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                self.wfile.write(part)
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError, OSError):
            pass

    def do_GET(self):
        if mode_get(self):
            return
        parsed = urlsplit(self.path)
        if parsed.path == "/api/ros/state":
            self.respond_json(200, ros_status())
            return
        if parsed.path.startswith("/api/ros/grid/"):
            layer = parsed.path.removeprefix("/api/ros/grid/")
            if layer not in ROS_GRIDS:
                self.respond_json(404, {"error": "Unknown occupancy grid layer"})
                return
            with ROS_LOCK:
                item = ROS_GRIDS[layer]
                self.respond_json(200, {"seq": item["seq"], "grid": item["grid"]})
            return
        if parsed.path == "/api/cameras":
            self.respond_json(200, {"cameras": camera_status(), "stream_urls": {name: "/api/camera/" + name + "/stream.mjpg" for name in CAMERA_NAMES}})
            return
        for name in CAMERA_NAMES:
            if parsed.path == "/api/camera/" + name + "/stream.mjpg":
                self.stream_camera(name)
                return
            if parsed.path == "/api/camera/" + name + "/snapshot.jpg":
                with CAMERA_CONDITION:
                    jpeg = CAMERA_FEEDS[name]["jpeg"]
                if not jpeg:
                    self.respond_json(404, {"error":"No frame received yet"})
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                self.wfile.write(jpeg)
                return
        if parsed.path == "/api/telemetry":
            try:
                from urllib.parse import parse_qs
                since = int(parse_qs(parsed.query).get("after", ["0"])[0])
            except (ValueError, TypeError):
                self.respond_json(400, {"error": "Invalid after sequence"})
                return
            with LOCK:
                seq = TELEMETRY["seq"]
                age = (time.monotonic() - TELEMETRY["last_mono"]) if seq else None
                online = age is not None and age < 2.0
                rows = [a for a in TELEMETRY["samples"] if a["seq"] > since]
                if since <= 0:
                    rows = rows[-1:]  # Initial live pose becomes the zero of field registration.
                self.respond_json(200, {"online": online, "age_s": round(age, 3) if age is not None else None,
                                        "seq": seq, "samples": rows[-150:]})
            return
        if parsed.path == "/api/scene":
            with LOCK:
                self.respond_json(200, dict(SHARED))
            return
        if parsed.path == "/api/build":
            self.respond_json(200, {"version": BUILD_VERSION, "build": "v13-sim2real-operator-console", "planner": ["astar", "dijkstra", "dstar"], "obstacles_max": 2, "grid_step_m": 0.48, "grid_shape": [5,5], "cell_numbering": "snake_0_to_24_bottom_left", "blocking": "one-grid-cell", "telemetry_api": "/api/telemetry", "camera_api":"/api/cameras", "ros_api":"/api/ros/state"})
            return
        if parsed.path == "/api/health":
            self.respond_json(200, {"ok": True, "service": "competition-v13-simulator", "shared": True, "version": BUILD_VERSION})
            return
        if parsed.path.startswith("/api/"):
            self.respond_json(404, {"error": "API 不存在"})
            return
        return super().do_GET()

    def do_POST(self):
        if mode_post(self):
            return
        path = urlsplit(self.path).path
        if path in ("/api/ros/state", "/api/ros/grid"):
            if not is_loopback(self.client_address[0]):
                self.respond_json(403, {"error": "ROS2 data publishing is localhost-only"})
                return
            try:
                payload = read_json_body(self, ROS_GRID_JSON_LIMIT if path.endswith("/grid") else ROS_JSON_LIMIT)
                if not isinstance(payload, dict):
                    raise ValueError("Expected JSON object")
                if path.endswith("/grid"):
                    layer, grid = validate_grid(payload)
                    with ROS_LOCK:
                        ROS_GRIDS[layer]["seq"] += 1
                        ROS_GRIDS[layer]["last_mono"] = time.monotonic()
                        ROS_GRIDS[layer]["grid"] = grid
                        seq = ROS_GRIDS[layer]["seq"]
                else:
                    # Strictly limit the exposed message surface. No commands or actuator control.
                    allowed = ("display_frame", "scan", "robot", "odom", "paths", "markers", "cmd_vel", "tf", "topics", "nodes", "nav", "diagnostics", "bridge")
                    snapshot = {k: payload[k] for k in allowed if k in payload}
                    if not isinstance(snapshot.get("topics",{}),dict) or not isinstance(snapshot.get("paths",{}),dict):
                        raise ValueError("Malformed ROS state")
                    with ROS_LOCK:
                        ROS_STATE["seq"] += 1
                        ROS_STATE["last_mono"] = time.monotonic()
                        ROS_STATE["snapshot"] = snapshot
                        seq = ROS_STATE["seq"]
            except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                self.respond_json(400, {"error": str(exc)[:160]})
                return
            self.respond_json(200, {"ok": True, "seq": seq})
            return
        for name in CAMERA_NAMES:
            if path != "/api/camera/" + name + "/frame":
                continue
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc.lower() != self.headers.get("Host", "").lower():
                self.respond_json(403, {"error":"Origin rejected"})
                return
            if not secrets.compare_digest(self.headers.get("X-Camera-Token", ""), CAMERA_TOKEN):
                self.respond_json(403, {"error":"Invalid camera publisher token"})
                return
            if self.headers.get("Content-Type", "").split(";",1)[0].strip().lower() != "image/jpeg":
                self.respond_json(415, {"error":"Expected image/jpeg"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except (TypeError,ValueError):
                length = 0
            if not 64 <= length <= MAX_JPEG_BYTES:
                self.respond_json(413, {"error":"Invalid JPEG size"})
                return
            jpeg = self.rfile.read(length)
            if len(jpeg) != length or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                self.respond_json(400, {"error":"Invalid JPEG frame"})
                return
            with CAMERA_CONDITION:
                feed = CAMERA_FEEDS[name]
                feed["jpeg"] = jpeg
                feed["seq"] += 1
                feed["last_mono"] = time.monotonic()
                feed["fps_times"].append(feed["last_mono"])
                feed["source"] = ("OpenMV H7 Plus USB" if name == "openmv" else "Raspberry Pi 5 CSI")
                CAMERA_CONDITION.notify_all()
                seq = feed["seq"]
            self.respond_json(200, {"ok": True, "camera": name, "seq": seq})
            return
        if urlsplit(self.path).path == "/api/telemetry":
            # Hardware telemetry is accepted ONLY from this host, not from LAN / public tunnel visitors.
            try:
                if not ipaddress.ip_address(self.client_address[0]).is_loopback:
                    self.respond_json(403, {"error": "Telemetry publishing is localhost-only"})
                    return
            except ValueError:
                self.respond_json(403, {"error": "Invalid client address"})
                return
            try:
                if self.headers.get("Content-Type", "").split(";",1)[0].strip().lower() != "application/json":
                    raise ValueError("Content-Type must be application/json")
                size=int(self.headers.get("Content-Length", "0"))
                if not 0<size<4096: raise ValueError("Telemetry JSON too large or empty")
                obj=json.loads(self.rfile.read(size))
                if not isinstance(obj,dict): raise ValueError("Sample must be an object")
                row={}
                for k, max_abs in (("x",100),("y",100),("yaw",10000),("vx",10),("vy",10),("wz",100)):
                    val=float(obj[k])
                    if not math.isfinite(val) or abs(val)>max_abs: raise ValueError("Invalid "+k)
                    row[k]=val
                row["frame"]=str(obj.get("frame","odom"))[:64]
                row["source"]=str(obj.get("source","ros2_odom"))[:64]
            except (ValueError,TypeError,KeyError,json.JSONDecodeError) as exc:
                self.respond_json(400,{"error":str(exc)[:140]})
                return
            with LOCK:
                TELEMETRY["seq"]+=1
                row["seq"]=TELEMETRY["seq"]
                row["received_at"]=time.time()
                TELEMETRY["last_mono"]=time.monotonic()
                TELEMETRY["samples"].append(row)
                self.respond_json(200,{"ok":True,"seq":row["seq"]})
            return
        if urlsplit(self.path).path != "/api/scene":
            self.respond_json(404, {"error": "API 不存在"})
            return
        origin = self.headers.get("Origin")
        if origin and urlsplit(origin).netloc.lower() != self.headers.get("Host", "").lower():
            self.respond_json(403, {"error": "跨站写入被拒绝"})
            return
        if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
            self.respond_json(415, {"error": "仅接受 JSON"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= MAX_BODY:
                self.respond_json(413, {"error": "上传数据超限"})
                return
            incoming = json.loads(self.rfile.read(length))
            payload = incoming.get("scene")
            if not isinstance(payload, dict) or int(payload.get("scenarioVersion",0)) < 10:
                raise ValueError("Outdated browser scene: reload the V10 page with Ctrl+Shift+R")
            new_scene = validate_scene(payload)
        except (ValueError, TypeError, KeyError, UnicodeError, json.JSONDecodeError) as exc:
            self.respond_json(400, {"error": str(exc)[:120]})
            return

        with LOCK:
            SHARED["revision"] += 1
            SHARED["scene"] = new_scene
            DATA.mkdir(parents=True, exist_ok=True)
            temp = SHARED_FILE.with_suffix(".tmp")
            try:
                temp.write_text(json.dumps(SHARED, ensure_ascii=False, indent=2), encoding="utf-8")
                os.replace(temp, SHARED_FILE)
            except OSError as exc:
                self.respond_json(500, {"error": f"场景保存失败：{exc}"})
                return
            self.respond_json(200, {"ok": True, "revision": SHARED["revision"]})

    def log_message(self, fmt, *args):
        if self.path.startswith(("/api/scene", "/api/telemetry", "/api/camera/", "/api/cameras", "/api/ros/")):
            return  # 多人同步每秒产生请求，不刷屏
        return super().log_message(fmt, *args)


def main():
    parser = argparse.ArgumentParser(description="工创 V13.0 Sim2Real控制台与完整底盘诊断 HTTP 服务")
    parser.add_argument("--port", type=int, default=8080, help="监听端口（默认 8080）")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("端口必须在 1–65535 内")
    try:
        server = ThreadingHTTPServer((args.host, args.port), Handler)
        server.daemon_threads = True
        server.keyboard_autostart_enabled = True  # Only local /keyboard GET may trigger ROS2 subscriber.
    except OSError as exc:
        parser.exit(1, f"无法启动 HTTP 服务：{exc}\n可能端口已被占用，可改用 --port 8081。\n")
    print("\n" + "═" * 58, flush=True)
    print(" Competition V11.0 simulator SERVER V11.0.0 is running", flush=True)
    print("═" * 58, flush=True)
    print(f"  On this computer: http://127.0.0.1:{args.port}/", flush=True)
    if args.host == "0.0.0.0":
        iplist = local_ipv4s()
        for ip in iplist:
            print(f"  On the same Wi-Fi/LAN: http://{ip}:{args.port}/", flush=True)
        if not iplist:
            print("  Find the computer LAN IP and use http://LAN-IP:8080/", flush=True)
    print("  ALGORITHMS: A* / Dijkstra / D* Lite", flush=True)
    print("  REAL-TIME: launch start_ros_bridge.sh when /odom is published", flush=True)
    print("  CAMERAS: OpenMV H7 Plus USB + Raspberry Pi 5 77deg via /api/cameras", flush=True)
    print("  Camera upload token is stored locally: data/camera_token.txt", flush=True)
    print("  BUILD CHECK: http://127.0.0.1:%d/api/build" % args.port, flush=True)
    print("  Shared scene sync is enabled for all visitors", flush=True)
    print("  Scenes are saved in data/shared_scene.json across restarts", flush=True)
    print("  Stop the server: press Ctrl+C in this terminal", flush=True)
    print("  For external networks: run bash start_public_tunnel.sh", flush=True)
    print("═" * 58 + "\n", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nServer stopped.", flush=True)
    finally:
        stop_keyboard_feedback_observer()
        server.server_close()

if __name__ == "__main__":
    main()
