cmake_minimum_required(VERSION 3.8)
project(car_web_sim)
find_package(ament_cmake REQUIRED)
# sim2real is a separate project; do not install/copy/move its source.
ament_package()
