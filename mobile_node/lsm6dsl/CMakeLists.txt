# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(DT1)

FILE(GLOB app_sources src/*.c)

FILE(GLOB lib_sources ../../mylib/*.c)

# build targets
target_sources(app PRIVATE ${app_sources} ${lib_sources})

# header file location
target_include_directories(app PRIVATE ../../include)
