# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
# Sysbuild entry: custom boards live under this application's boards/ tree.
# Required when building from the west workspace top (e.g. west build ... NEO).

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)