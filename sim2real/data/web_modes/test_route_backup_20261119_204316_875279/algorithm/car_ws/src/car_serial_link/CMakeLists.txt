cmake_minimum_required(VERSION 3.8)
project(car_serial_link)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(car_interfaces REQUIRED)

add_library(car_link_codec STATIC src/protocol.cpp)
target_include_directories(car_link_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(car_link_codec PUBLIC cxx_std_17)

add_executable(car_serial_bridge src/serial_bridge_node.cpp)
target_compile_features(car_serial_bridge PRIVATE cxx_std_17)
target_link_libraries(car_serial_bridge car_link_codec)
ament_target_dependencies(car_serial_bridge
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs diagnostic_msgs car_interfaces)

add_executable(car_serial_mock_mcu src/mock_mcu_node.cpp)
target_compile_features(car_serial_mock_mcu PRIVATE cxx_std_17)
target_link_libraries(car_serial_mock_mcu car_link_codec util)
ament_target_dependencies(car_serial_mock_mcu rclcpp)

install(TARGETS car_serial_bridge car_serial_mock_mcu DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES include/car_serial_link/protocol.hpp DESTINATION include/car_serial_link)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_car_link_codec tests/test_protocol.cpp)
  target_link_libraries(test_car_link_codec car_link_codec)
  add_test(NAME car_link_codec_test COMMAND test_car_link_codec)
endif()
ament_package()
