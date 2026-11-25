# Raspberry Pi 5: install ROS 2 Humble in a Jammy ARM64-compatible container,
# keeping Raspberry Pi OS and its CSI / display drivers on the host.
# Base image is multi-architecture; on Pi5 it automatically selects arm64.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Only the current repo's read-only ULog43 bridge + its diagnostic tools.
# Explicit dependencies so the image does not depend on ambient apt state.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
      ros-humble-ament-cmake \
      ros-humble-rosidl-default-generators \
      ros-humble-ros2launch ros-humble-launch-xml \
      ros-humble-rclcpp \
      ros-humble-std-msgs ros-humble-geometry-msgs \
      ros-humble-nav-msgs ros-humble-sensor-msgs \
      ros-humble-diagnostic-msgs \
      ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["/bin/bash"]
