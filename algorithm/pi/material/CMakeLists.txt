cmake_minimum_required(VERSION 3.16)
project(pi5_material_visual_servo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Stack on algorithm/pi/QR. Reuse its exact four-group task-code parser.
set(PI_QR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../QR")
if(NOT EXISTS "${PI_QR_ROOT}/src/task_code.cpp")
    message(FATAL_ERROR "algorithm/pi/QR is required; check out PR #13 first")
endif()

add_library(pi_qr_code STATIC "${PI_QR_ROOT}/src/task_code.cpp")
target_include_directories(pi_qr_code PUBLIC "${PI_QR_ROOT}/include")

add_library(pi_material_core STATIC src/core.cpp)
target_include_directories(pi_material_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(pi_material_core PUBLIC pi_qr_code)
target_compile_options(pi_material_core PRIVATE -Wall -Wextra -Wpedantic)

# Core C++ unit test is independent of OpenCV and CSI camera hardware.
include(CTest)
if(BUILD_TESTING)
    add_executable(test_material_core tests/test_core.cpp)
    target_link_libraries(test_material_core PRIVATE pi_material_core)
    target_compile_options(test_material_core PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME material_sequence_and_servo COMMAND test_material_core)
endif()

option(PI_MATERIAL_WITH_OPENCV "Build OpenCV camera detection + GUI" ON)
if(PI_MATERIAL_WITH_OPENCV)
    find_package(OpenCV 4.5 REQUIRED COMPONENTS
        core imgproc highgui imgcodecs videoio)

    add_library(pi_material_cv STATIC src/detector.cpp src/camera.cpp)
    target_include_directories(pi_material_cv PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include" ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(pi_material_cv PUBLIC
        pi_material_core ${OpenCV_LIBS})
    target_compile_options(pi_material_cv PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(pi_material src/main.cpp)
    target_link_libraries(pi_material PRIVATE pi_material_cv)
    target_compile_options(pi_material PRIVATE -Wall -Wextra -Wpedantic)

    if(BUILD_TESTING)
        add_executable(test_material_detector tests/test_detector.cpp)
        target_link_libraries(test_material_detector PRIVATE pi_material_cv)
        target_compile_options(test_material_detector PRIVATE -Wall -Wextra -Wpedantic)
        add_test(NAME material_synthetic_color_detection COMMAND test_material_detector)
    endif()
endif()
