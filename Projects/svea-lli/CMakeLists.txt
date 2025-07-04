# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(svea_lli_control C CXX)

# Include directories
target_include_directories(app PRIVATE include)

# Source files
target_sources(app PRIVATE
    src/main.c
    src/control.c
    src/rc_input.c
    src/remote.c
    src/servo.c
    src/sensors.c
    src/ros_iface.c
)