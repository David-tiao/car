#!/usr/bin/env python3
"""Generate an explicitly provisional URDF and a Fortress static scene.
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


def robot_description():
    robot = ET.Element("robot", {"name": "car_preview"})
    chassis = sub(robot, "link", name="base_link")
    urdf_part(chassis, "box", BODY, (0, 0, 0.075), (0, 0, 0), BODY_COLOR)
    for suffix, x, y in WHEELS:
        link = sub(robot, "link", name="wheel_" + suffix + "_link")
        urdf_part(link, "cylinder", (R, WHEEL_WIDTH), (0, 0, 0),
                  (math.pi / 2, 0, 0), BLACK)
        urdf_joint(robot, "wheel_" + suffix + "_joint", "base_link",
                   link.get("name"), (x, y, R), "continuous")
    # No fictitious real laser_frame, camera extrinsic or arm joint.
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
    if mode not in ("preview", "mirror"):
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


def world_description(zones, obstacles=(), mode="preview"):
    validate(zones, obstacles, mode)
    sdf = ET.Element("sdf", {"version": "1.8"})
    world = sub(sdf, "world", name="car_" + mode)
    for filename, name in (("physics", "Physics"), ("user-commands", "UserCommands"),
                           ("scene-broadcaster", "SceneBroadcaster")):
        sub(world, "plugin", filename="ignition-gazebo-" + filename + "-system",
            name="ignition::gazebo::systems::" + name)
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
    # A static visual-only car: preview is NOT a dynamics simulation.
    # Mirror starts outside the field until a fresh real pose is available.
    xyz = (2.25, 2.25, 0.001) if mode == "preview" else (-0.4, -0.4, 0.001)
    car = static_model(world, "car_" + mode + "_model", xyz)
    sdf_part(car, "chassis", "box", BODY, (0, 0, 0.075, 0, 0, 0), BODY_COLOR, False)
    for suffix, x, y in WHEELS:
        sdf_part(car, "wheel_" + suffix, "cylinder", (R, WHEEL_WIDTH),
                 (x, y, R, math.pi/2, 0, 0), BLACK, False)
    return sdf


def xml_bytes(root):
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--zones", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--mode", choices=("preview", "mirror"), default="preview")
    p.add_argument("--obstacle", type=float, nargs=2, action="append", default=[])
    args = p.parse_args()
    try:
        zones = json.loads(args.zones.read_text(encoding="utf-8"))
        outputs = {"car_preview.urdf": xml_bytes(robot_description()),
                   "car_" + args.mode + ".sdf":
                       xml_bytes(world_description(zones, args.obstacle, args.mode))}
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
