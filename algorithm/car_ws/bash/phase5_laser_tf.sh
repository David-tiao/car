#!/usr/bin/env bash
# Publish only a MEASURED static extrinsic base_link->laser_frame.
# Usage: bash phase5_laser_tf.sh X Y Z ROLL PITCH YAW (metres/radians).
# Does not invent map->odom or dynamic odom->base_link.
set -Eeuo pipefail
if (($# != 6));then
 echo 'Usage: bash phase5_laser_tf.sh x_m y_m z_m roll_rad pitch_rad yaw_rad' >&2
 echo 'Measure the real lidar extrinsic first; do not use guessed values on hardware.' >&2
 exit 2
fi
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
exec ros2 run tf2_ros static_transform_publisher \
  --x "$1" --y "$2" --z "$3" \
  --roll "$4" --pitch "$5" --yaw "$6" \
  --frame-id base_link --child-frame-id laser_frame
