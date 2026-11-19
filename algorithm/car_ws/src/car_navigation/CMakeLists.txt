cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(car_navigation_core src/navigation_core.cpp)
target_include_directories(car_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(car_navigation_core PUBLIC cxx_std_17)

add_executable(car_map_publisher src/map_publisher_node.cpp)
target_compile_features(car_map_publisher PRIVATE cxx_std_17)
target_link_libraries(car_map_publisher car_navigation_core)
ament_target_dependencies(car_map_publisher rclcpp nav_msgs)

add_executable(car_path_preview src/path_preview_node.cpp)
target_compile_features(car_path_preview PRIVATE cxx_std_17)
target_link_libraries(car_path_preview car_navigation_core)
ament_target_dependencies(car_path_preview rclcpp nav_msgs geometry_msgs std_msgs)

add_executable(car_navigation_monitor src/scan_monitor_node.cpp)
target_compile_features(car_navigation_monitor PRIVATE cxx_std_17)
ament_target_dependencies(car_navigation_monitor rclcpp nav_msgs sensor_msgs diagnostic_msgs tf2 tf2_ros)

add_executable(car_map_probe src/map_probe.cpp)
target_compile_features(car_map_probe PRIVATE cxx_std_17)
target_link_libraries(car_map_probe car_navigation_core)

add_executable(car_mock_scan src/mock_scan_node.cpp)
target_compile_features(car_mock_scan PRIVATE cxx_std_17)
ament_target_dependencies(car_mock_scan rclcpp sensor_msgs)

install(TARGETS car_map_publisher car_path_preview car_navigation_monitor car_mock_scan car_map_probe
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
