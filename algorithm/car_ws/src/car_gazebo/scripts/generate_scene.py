#!/usr/bin/env python3
"""Generate an explicitly provisional URDF and Fortress/Harmonic scene.
No real TF, actuator, simulated odometry, or lidar publisher is started here.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

# Wheel radius and axle half-spacings: STM32F4/App/app_runtime.c.
R, LX, LY = 0.04, 0.093, 0.11
# The remaining geometry is a PREVIEW assumption, not measured mechanical CAD.
BODY = (0.24, 0.18, 0.05)
WHEEL_WIDTH = 0.03
BODY_Z = 0.060
# Complete provisional wheel/body envelope.
SIM_ENVELOPE = (max(BODY[0], 2*(LX+R)), max(BODY[1], 2*LY+WHEEL_WIDTH), 0.085)
LASER_Z = 0.092
SIM_DOMAIN = "84"
SIM_PREFIX = "/car_gz"
SIM_MODEL = "car_sim_model"
SIM_MAX_LINEAR = 0.20
SIM_MAX_ANGULAR = 0.50
SIM_CMD_TTL = 0.30
SIM_DATA_TTL = 0.50

def limited_twist(vx, vy, wz):
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return (0., 0., 0.)
    magnitude = math.hypot(vx, vy)
    scale = min(1., SIM_MAX_LINEAR / magnitude) if magnitude else 1.
    return (vx * scale, vy * scale,
            max(-SIM_MAX_ANGULAR, min(SIM_MAX_ANGULAR, wz)))


def guarded_twist(command, command_age, odom_age, scan_age, clock_age):
    if (not 0 <= command_age <= SIM_CMD_TTL or
            any(not 0 <= age <= SIM_DATA_TTL for age in
                (odom_age, scan_age, clock_age))):
        return (0., 0., 0.)
    return limited_twist(*command)


def wheel_rates(vx, vy, wz):
    """Ideal X-pattern mecanum, visualization only."""
    a = (LX + LY) * wz
    return ((vx - vy - a)/R, (vx + vy + a)/R,
            (vx + vy - a)/R, (vx - vy + a)/R)

WHEELS = (("fl", LX, LY), ("fr", LX, -LY),
          ("rl", -LX, LY), ("rr", -LX, -LY))
GRAY = "0.45 0.45 0.45 1"
BLUE = "0.05 0.20 0.90 1"
YELLOW = "0.94 0.90 0.56 1"
WHITE = "0.88 0.88 0.88 1"
BLACK = "0.05 0.05 0.05 1"
BODY_COLOR = "0.14 0.55 0.68 1"


def sub(parent, tag, value=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if value is not None:
        element.text = str(value)
    return element


def vec(values):
    return " ".join(format(v, ".9g") for v in values)


def geometry(parent, shape, dims, urdf=False):
    g = sub(parent, "geometry")
    if urdf:
        if shape == "box":
            sub(g, shape, size=vec(dims))
        else:
            sub(g, shape, radius=str(dims[0]), length=str(dims[1]))
    elif shape == "box":
        sub(sub(g, shape), "size", vec(dims))
    else:
        item = sub(g, shape)
        sub(item, "radius", dims[0])
        sub(item, "length", dims[1])


def urdf_part(link, shape, dims, xyz, rpy, color):
    for tag in ("visual", "collision"):
        item = sub(link, tag)
        sub(item, "origin", xyz=vec(xyz), rpy=vec(rpy))
        geometry(item, shape, dims, urdf=True)
        if tag == "visual":
            sub(sub(item, "material", name=link.get("name") + "_color"),
                "color", rgba=color)


def urdf_joint(robot, name, parent, child, xyz, kind="fixed"):
    j = sub(robot, "joint", name=name, type=kind)
    sub(j, "parent", link=parent)
    sub(j, "child", link=child)
    sub(j, "origin", xyz=vec(xyz), rpy="0 0 0")
    if kind == "continuous":
        sub(j, "axis", xyz="0 1 0")


def robot_description(sim=False):
    robot = ET.Element("robot", {"name": "car_sim" if sim else "car_preview"})
    chassis = sub(robot, "link", name="base_link")
    urdf_part(chassis, "box", BODY, (0, 0, BODY_Z), (0, 0, 0), BODY_COLOR)
    for suffix, x, y in WHEELS:
        link = sub(robot, "link", name="wheel_" + suffix + "_link")
        urdf_part(link, "cylinder", (R, WHEEL_WIDTH), (0, 0, 0),
                  (math.pi / 2, 0, 0), BLACK)
        urdf_joint(robot, "wheel_" + suffix + "_joint", "base_link",
                   link.get("name"), (x, y, R), "continuous")
    if sim:
        laser = sub(robot, "link", name="laser_frame")
        urdf_joint(robot, "laser_joint", "base_link", laser.get("name"),
                   (0, 0, LASER_Z))
    # The real preview has no fictitious laser extrinsic or arm joint.
    return robot


def sdf_part(link, name, shape, dims, pose, color, collision):
    visual = sub(link, "visual", name=name + "_visual")
    sub(visual, "pose", vec(pose))
    geometry(visual, shape, dims)
    material = sub(visual, "material")
    sub(material, "ambient", color)
    sub(material, "diffuse", color)
    if collision:
        item = sub(link, "collision", name=name + "_collision")
        sub(item, "pose", vec(pose))
        geometry(item, shape, dims)


def static_model(world, name, xyz):
    model = sub(world, "model", name=name)
    sub(model, "static", "true")
    sub(model, "pose", vec((*xyz, 0, 0, 0)))
    return sub(model, "link", name="link")


def rectangle(world, name, rect, color):
    x0, y0, x1, y1 = rect
    link = static_model(world, name, ((x0+x1)/2, (y0+y1)/2, 0.0005))
    sdf_part(link, name, "box", (x1-x0, y1-y0, 0.001),
             (0, 0, 0, 0, 0, 0), color, False)


def point_rect_distance(x, y, rect):
    a, b, c, d = rect
    return math.hypot(max(a-x, 0, x-c), max(b-y, 0, y-d))


def validate(zones, obstacles, mode):
    if mode not in ("preview", "mirror", "sim"):
        raise ValueError("unknown mode")
    if tuple(zones["field_size"]) != (2.4, 2.4):
        raise ValueError("unexpected competition field size")
    for group, count in (("yellow", 4), ("blue", 2), ("work", 2)):
        if len(zones[group]) != count:
            raise ValueError("invalid zone count: " + group)
        for x0, y0, x1, y1 in zones[group]:
            if not (0 <= x0 < x1 <= 2.4 and 0 <= y0 < y1 <= 2.4):
                raise ValueError("invalid rectangle: " + group)
    disk = zones["raw_disk"]
    cx, cy = disk["center"]
    radius = disk["radius"]
    if not (math.isfinite(cx) and math.isfinite(cy) and
            0 <= cx <= 2.4 and 0 <= cy <= 2.4 and radius == 0.15):
        raise ValueError("invalid raw disk")
    if mode == "mirror" and obstacles:
        raise ValueError("mirror must not invent real obstacle positions")
    for i, (x, y) in enumerate(obstacles):
        if not (0.025 <= x <= 2.375 and 0.025 <= y <= 2.375):
            raise ValueError("obstacle outside field")
        if any(point_rect_distance(x, y, r) <= 0.025
               for r in zones["yellow"] + zones["blue"] + zones["work"]):
            raise ValueError("obstacle overlaps forbidden or work zone")
        if math.hypot(x-cx, y-cy) <= radius+0.025:
            raise ValueError("obstacle overlaps raw disk")
        if any(math.hypot(x-a, y-b) <= 0.05 for a, b in obstacles[:i]):
            raise ValueError("obstacles overlap")


def sim_model(world, start):
    # Model-level velocity control is an omnidirectional approximation.
    # Wheel visuals are fixed; mass and collision envelope are provisional.
    model = sub(world, "model", name=SIM_MODEL)
    sub(model, "pose", vec((*start, 0.001, 0, 0, 0)))
    link = sub(model, "link", name="base_link")
    inertial = sub(link, "inertial")
    sub(inertial, "pose", vec((0, 0, SIM_ENVELOPE[2]/2, 0, 0, 0)))
    mass = 2.0
    dims = SIM_ENVELOPE
    sub(inertial, "mass", mass)
    inertia = sub(inertial, "inertia")
    for axis, moment in zip(("ixx", "iyy", "izz"),
                            (mass*(dims[1]**2+dims[2]**2)/12,
                             mass*(dims[0]**2+dims[2]**2)/12,
                             mass*(dims[0]**2+dims[1]**2)/12)):
        sub(inertia, axis, moment)
    for axis in ("ixy", "ixz", "iyz"):
        sub(inertia, axis, 0)
    envelope = sub(link, "collision", name="provisional_envelope")
    sub(envelope, "pose", vec((0, 0, SIM_ENVELOPE[2]/2, 0, 0, 0)))
    geometry(envelope, "box", dims)
    sdf_part(link, "chassis", "box", BODY, (0, 0, BODY_Z, 0, 0, 0), BODY_COLOR, False)
    for suffix, x, y in WHEELS:
        sdf_part(link, "wheel_" + suffix, "cylinder", (R, WHEEL_WIDTH),
                 (x, y, R, math.pi/2, 0, 0), BLACK, False)
    lidar = sub(link, "sensor", name="gpu_lidar", type="gpu_lidar")
    sub(lidar, "pose", vec((0, 0, LASER_Z, 0, 0, 0)))
    sub(lidar, "topic", SIM_PREFIX + "/scan_raw")
    sub(lidar, "ignition_frame_id", "laser_frame")
    sub(lidar, "always_on", "true")
    sub(lidar, "update_rate", 10)
    ray = sub(lidar, "ray")
    horizontal = sub(sub(ray, "scan"), "horizontal")
    sub(horizontal, "samples", 720)
    sub(horizontal, "resolution", 1)
    sub(horizontal, "min_angle", -math.pi)
    sub(horizontal, "max_angle", math.pi)
    ranges = sub(ray, "range")
    sub(ranges, "min", 0.05)
    sub(ranges, "max", 4.0)
    sub(ranges, "resolution", 0.005)
    drive = sub(model, "plugin", filename="libignition-gazebo-velocity-control-system.so",
                name="ignition::gazebo::systems::VelocityControl")
    sub(drive, "topic", SIM_PREFIX + "/drive_cmd")
    sub(drive, "initial_linear", "0 0 0")
    sub(drive, "initial_angular", "0 0 0")
    odom = sub(model, "plugin", filename="libignition-gazebo-odometry-publisher-system.so",
               name="ignition::gazebo::systems::OdometryPublisher")
    for k, v in (("odom_frame", "odom"), ("robot_base_frame", "base_link"),
                 ("odom_topic", SIM_PREFIX + "/odom_raw"),
                 ("tf_topic", SIM_PREFIX + "/unused_gz_tf"),
                 ("odom_publish_frequency", 30), ("dimensions", 2)):
        sub(odom, k, v)
    return model


def world_description(zones, obstacles=(), mode="preview", start=(2.25, 2.25), backend="fortress"):
    if backend not in ("fortress", "harmonic"):
        raise ValueError("unsupported SDF backend")
    validate(zones, obstacles, mode)
    if mode == "sim":
        x, y = start
        hx, hy = SIM_ENVELOPE[0]/2, SIM_ENVELOPE[1]/2
        if not (hx <= x <= 2.4-hx and hy <= y <= 2.4-hy):
            raise ValueError("SIM start outside field")
        footprint = (x-hx, y-hy, x+hx, y+hy)
        if any(not (footprint[2] <= a or footprint[0] >= c or
                    footprint[3] <= b or footprint[1] >= d)
               for a, b, c, d in zones["yellow"] + zones["work"]):
            raise ValueError("SIM start overlaps forbidden zone")
        cx, cy = zones["raw_disk"]["center"]
        if point_rect_distance(cx, cy, footprint) <= zones["raw_disk"]["radius"]:
            raise ValueError("SIM start overlaps raw disk")
        if any(point_rect_distance(a, b, footprint) <= 0.025 for a,b in obstacles):
            raise ValueError("SIM start overlaps obstacle")
    sdf = ET.Element("sdf", {"version": "1.8"})
    world = sub(sdf, "world", name="car_" + mode)
    for filename, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                           ("scene-broadcaster", "SceneBroadcaster")):
        sub(world, "plugin", filename="ignition-gazebo-" + filename + "-system",
            name="ignition::gazebo::systems::" + name)
    if mode == "sim":
        sensors = sub(world, "plugin", filename="libignition-gazebo-sensors-system.so",
                      name="ignition::gazebo::systems::Sensors")
        sub(sensors, "render_engine", "ogre2")
    light = sub(world, "light", name="sun", type="directional")
    sub(light, "pose", "0 0 5 0 0 0")
    sub(light, "direction", "-0.3 0.2 -1")
    sub(light, "diffuse", "0.9 0.9 0.9 1")
    floor = static_model(world, "field", (1.2, 1.2, -0.01))
    sdf_part(floor, "floor", "box", (2.4, 2.4, 0.02),
             (0, 0, 0, 0, 0, 0), GRAY, True)
    for group, color in (("yellow", YELLOW), ("blue", BLUE), ("work", WHITE)):
        for i, rect in enumerate(zones[group], 1):
            rectangle(world, group + "_" + str(i), rect, color)
    cx, cy = zones["raw_disk"]["center"]
    disk = static_model(world, "raw_disk", (cx, cy, 0.045))
    sdf_part(disk, "disk", "cylinder", (zones["raw_disk"]["radius"], 0.09),
             (0, 0, 0, 0, 0, 0), WHITE, True)
    for i, (x, y) in enumerate(obstacles, 1):
        link = static_model(world, "test_obstacle_" + str(i), (x, y, 0.05))
        sdf_part(link, "obstacle", "cylinder", (0.025, 0.10),
                 (0, 0, 0, 0, 0, 0), BLACK, True)
    if mode == "sim":
        sim_model(world, start)
        return backend_scene(sdf, backend)
    # A static visual-only car: preview is NOT a dynamics simulation.
    # Mirror starts outside the field until a fresh real pose is available.
    xyz = (2.25, 2.25, 0.001) if mode == "preview" else (-0.4, -0.4, 0.001)
    car = static_model(world, "car_" + mode + "_model", xyz)
    sdf_part(car, "chassis", "box", BODY, (0, 0, BODY_Z, 0, 0, 0), BODY_COLOR, False)
    for suffix, x, y in WHEELS:
        sdf_part(car, "wheel_" + suffix, "cylinder", (R, WHEEL_WIDTH),
                 (x, y, R, math.pi/2, 0, 0), BLACK, False)
    return backend_scene(sdf, backend)


def backend_scene(sdf, backend):
    # Translate only the simulator integration names; geometry and parameters are shared.
    if backend == "harmonic":
        for plugin in sdf.findall(".//plugin"):
            filename = plugin.get("filename")
            name = plugin.get("name")
            plugin.set("filename", filename.removeprefix("lib").removesuffix(".so").replace("ignition-gazebo-", "gz-sim-", 1))
            plugin.set("name", name.replace("ignition::gazebo::", "gz::sim::", 1))
        for frame in sdf.findall(".//sensor/ignition_frame_id"):
            frame.tag = "gz_frame_id"
    return sdf


def xml_bytes(root):
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--zones", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--mode", choices=("preview", "mirror", "sim"), default="preview")
    p.add_argument("--backend", choices=("fortress", "harmonic"), default="fortress")
    p.add_argument("--start", type=float, nargs=2, default=(2.25, 2.25), help="SIM only; map x y, yaw=0")
    p.add_argument("--obstacle", type=float, nargs=2, action="append", default=[])
    args = p.parse_args()
    try:
        zones = json.loads(args.zones.read_text(encoding="utf-8"))
        if args.mode != "sim" and tuple(args.start) != (2.25, 2.25):
            raise ValueError("--start is SIM only")
        model_name = "car_sim.urdf" if args.mode == "sim" else "car_preview.urdf"
        outputs = {model_name: xml_bytes(robot_description(args.mode == "sim")),
                   "car_" + args.mode + ".sdf":
                       xml_bytes(world_description(zones, args.obstacle, args.mode, args.start, args.backend))}
        args.output.mkdir(parents=True, exist_ok=True)
        for name in outputs:
            if (args.output / name).exists():
                raise FileExistsError("refusing to overwrite " + name)
        for name, data in outputs.items():
            with (args.output / name).open("xb") as f:
                f.write(data)
            print(args.output / name)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        p.error(str(exc))


if __name__ == "__main__":
    main()
