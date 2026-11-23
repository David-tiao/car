cmake_minimum_required(VERSION 3.16)
project(pi5_qr_display LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Raspberry Pi OS: sudo apt install libopencv-dev cmake build-essential
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc objdetect
    highgui imgcodecs videoio)

add_library(pi_qr_task STATIC src/task_code.cpp)
target_include_directories(pi_qr_task PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_options(pi_qr_task PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pi_qr src/main.cpp)
target_link_libraries(pi_qr PRIVATE pi_qr_task ${OpenCV_LIBS})
target_include_directories(pi_qr PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(pi_qr PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_task_code tests/test_task_code.cpp)
    target_link_libraries(test_task_code PRIVATE pi_qr_task)
    target_compile_options(test_task_code PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME task_code_parser_and_confirmation COMMAND test_task_code)
endif()
