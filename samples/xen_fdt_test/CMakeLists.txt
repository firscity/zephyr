# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xen_fdt_test)

set(ZEPHYR_MODULES ../zephyr-xenlib)
include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../zephyr-xenlib/libfdt)
target_sources(app PRIVATE src/main.c)
