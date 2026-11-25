#!/usr/bin/env python3
"""Select a matching Gazebo simulator and ROS bridge ABI; never install packages."""
import argparse
import os
from pathlib import Path
import re
import subprocess

# gz-sim 6/Fortress: ignition-transport11, ignition-msgs8.
# gz-sim 8/Harmonic: gz-transport13, gz-msgs10.
BACKENDS = {
    "fortress": (("ign", "gazebo"), 6, "ignition-transport11", "ignition-msgs8",
                 "ignition.msgs", "IGN_PARTITION"),
    "harmonic": (("gz", "sim"), 8, "gz-transport13", "gz-msgs10",
                 "gz.msgs", "GZ_PARTITION"),
}


def bridge_backend(linker_output):
    if "not found" in linker_output:
        raise ValueError("ros_gz_bridge has unresolved shared libraries")
    matches = [name for name, (_, _, transport, msgs, _, _) in BACKENDS.items()
               if all(re.search(r"\blib" + lib + r"\.so(?:[.\s]|$)", linker_output)
                      for lib in (transport, msgs))]
    if len(matches) != 1:
        raise ValueError("unsupported or ambiguous ROS bridge ABI (need Fortress or Harmonic)")
    # Reject an executable loading any other supported transport ABI simultaneously.
    if any(name != matches[0] and re.search(r"\blib" + spec[2] + r"\.so", linker_output)
           for name, spec in BACKENDS.items()):
        raise ValueError("mixed Gazebo Transport ABIs")
    return matches[0]


def version_available(output, major):
    return any(int(v) == major for v in re.findall(r"(?<![\d.])(\d+)\.\d+\.\d+(?![\d.])", output))


def checked_output(args):
    try:
        return subprocess.run(args, check=True, capture_output=True, text=True,
                              timeout=15).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        raise ValueError(f"command unavailable or failed: {' '.join(map(str, args))}: {exc}") from exc


def select_backend(requested="auto", run=checked_output):
    if requested not in ("auto", *BACKENDS):
        raise ValueError("backend must be auto, fortress or harmonic")
    prefix = Path(run(["ros2", "pkg", "prefix", "ros_gz_bridge"]).strip())
    exe = prefix / "lib/ros_gz_bridge/parameter_bridge"
    if not exe.is_file() or not os.access(exe, os.X_OK):
        raise ValueError(f"missing ROS bridge executable: {exe}")
    actual = bridge_backend(run(["ldd", str(exe)]))
    if requested != "auto" and requested != actual:
        raise ValueError(f"requested {requested} but bridge links to {actual}; use matching ROS overlay/packages")
    command, major, *_ = BACKENDS[actual]
    if not version_available(run([*command, "--versions"]), major):
        raise ValueError(f"matching Gazebo Sim major {major} not installed for {actual}")
    return actual


def simulator_command(name):
    cmd, major, *_ = BACKENDS[name]
    return [*cmd, "--force-version", str(major)]


def bridge_arguments(name):
    msg = BACKENDS[name][4]
    return [
        f"/world/car_sim/clock@rosgraph_msgs/msg/Clock[{msg}.Clock",
        f"/car_gz/odom_raw@nav_msgs/msg/Odometry[{msg}.Odometry",
        f"/car_gz/scan_raw@sensor_msgs/msg/LaserScan[{msg}.LaserScan",
        f"/car_gz/drive_cmd@geometry_msgs/msg/Twist]{msg}.Twist",
        "--ros-args", "-r", "/world/car_sim/clock:=/clock",
    ]


def partition_environment(name, partition):
    # Both variables receive the same unique value to avoid inherited aliases.
    if name not in BACKENDS or not partition:
        raise ValueError("valid backend and partition required")
    return {"IGN_PARTITION": partition, "GZ_PARTITION": partition}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--backend", choices=("auto", *BACKENDS), default=os.environ.get("CAR_GAZEBO_BACKEND", "auto"))
    args = p.parse_args()
    try:
        print(select_backend(args.backend))
    except ValueError as exc:
        p.exit(2, f"ENV FAIL: {exc}\n")


if __name__ == "__main__":
    main()
