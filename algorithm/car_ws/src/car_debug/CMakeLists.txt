cmake_minimum_required(VERSION 3.8)
project(car_debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(car_data_monitor src/data_monitor_node.cpp)
target_include_directories(car_data_monitor PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(car_data_monitor PRIVATE cxx_std_17)
ament_target_dependencies(car_data_monitor
  rclcpp car_interfaces nav_msgs geometry_msgs diagnostic_msgs)

install(TARGETS car_data_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
