cmake_minimum_required(VERSION 3.8)
project(car_stm32_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(car_interfaces REQUIRED)

add_library(ulog43_parser src/ulog43_parser.cpp)
target_include_directories(ulog43_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ulog43_parser PUBLIC cxx_std_17)

add_executable(car_ulog43_bridge src/ulog43_bridge_node.cpp)
target_compile_features(car_ulog43_bridge PRIVATE cxx_std_17)
target_link_libraries(car_ulog43_bridge ulog43_parser)
ament_target_dependencies(car_ulog43_bridge
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2_ros car_interfaces)

install(TARGETS car_ulog43_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
