cmake_minimum_required(VERSION 3.5)
project(untitled)

include_directories(
        ~/Desktop/fpgaPF/f32c/src/examples/pf_serial/mr2015ws/src/tuw_mr2015/tuw_self_localization/include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES main.cpp
        ~/Desktop/fpgaPF/f32c/src/examples/pf_serial/mr2015ws/src/tuw_mr2015/tuw_self_localization/src/tuw_self_localization/rs232.c
        ~/Desktop/fpgaPF/f32c/src/examples/pf_serial/mr2015ws/src/tuw_mr2015/tuw_self_localization/src/tuw_self_localization/serial_support.cpp
        )

add_executable(untitled ${SOURCE_FILES})