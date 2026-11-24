cmake_minimum_required(VERSION 3.8)
project(car_gazebo)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/generate_scene.py scripts/mirror_pose.py scripts/sim_adapter.py scripts/follow_locked_route.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME car_gazebo_scene
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_scene.py)
endif()
ament_package()
