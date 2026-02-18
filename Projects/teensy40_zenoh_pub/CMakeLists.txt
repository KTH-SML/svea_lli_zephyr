cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/zenoh-pico
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(teensy40_zenoh_pub)

zephyr_compile_definitions(ZENOH_GENERIC)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_sources(app PRIVATE src/main.c)
