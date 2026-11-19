cmake_minimum_required(VERSION 3.8)
project(car_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisTelemetry.msg"
  "msg/WheelTelemetry.msg"
  "msg/MissionState.msg"
  "msg/Ulog43.msg"
  "msg/VisionEvent.msg"
  "msg/MechanismState.msg"
  "msg/CommandFeedback.msg"
  "msg/PowerTelemetry.msg"
  "msg/LinkStatus.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
