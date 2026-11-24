# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
# SDK V1.2.7 can export a bare "-lydlidar_sdk" without its custom install
# directory. Resolve the library from the selected config, not a system-wide
# search path (which could silently select an unrelated SDK).
get_filename_component(_car_sdk_libdir "${ydlidar_sdk_DIR}/../.." REALPATH)
unset(CAR_YDLIDAR_SDK_LIBRARY CACHE)
find_library(CAR_YDLIDAR_SDK_LIBRARY NAMES ydlidar_sdk
  PATHS "${_car_sdk_libdir}" ${YDLIDAR_SDK_LIBRARY_DIRS}
  NO_DEFAULT_PATH)
if(NOT CAR_YDLIDAR_SDK_LIBRARY)
  message(FATAL_ERROR "YDLIDAR SDK library missing for config: ${ydlidar_sdk_DIR}")
endif()
message(STATUS "CAR YDLIDAR SDK library: ${CAR_YDLIDAR_SDK_LIBRARY}")
set(_car_sdk_extra_libraries ${YDLIDAR_SDK_LIBRARIES})
list(REMOVE_ITEM _car_sdk_extra_libraries ydlidar_sdk -lydlidar_sdk)
# Remove duplicate absolute SDK entries while retaining pthread/rt and other
# dependencies supplied by the SDK package config.
list(FILTER _car_sdk_extra_libraries EXCLUDE REGEX "(^|/)libydlidar_sdk\\.(a|so)(\\..*)?$")
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

target_link_libraries(${PROJECT_NAME}_node
    "${CAR_YDLIDAR_SDK_LIBRARY}" ${_car_sdk_extra_libraries})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    ${PROJECT_NAME}_node ${PROJECT_NAME}_client
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params startup config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

