# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(simple_microros_publisher C CXX)

# Gather all source files
file(GLOB THREADS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/threads/**/**/*.c
)

target_sources(app PRIVATE
    src/main.c
    ${THREADS_SRC}
)