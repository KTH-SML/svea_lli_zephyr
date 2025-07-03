# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(svea-lli)

# Public headers for every source file in this application
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# List every .c (or .cpp) file to be built into the final firmware
target_sources(app PRIVATE
  src/rc_input.c
  src/remote.c
  src/servo.c
  src/ros_iface.c
  src/control.c
  src/sensor_imu.c
  src/main.c
)
