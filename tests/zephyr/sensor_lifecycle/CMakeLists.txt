cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_sensor_lifecycle)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(TRACKER "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH "Tracker checkout")
set(TWIM "$ENV{ZEPHYR_BASE}/drivers/i2c/i2c_nrfx_twim.c" CACHE FILEPATH "Actual TWIM driver")
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_command(
  OUTPUT ${GENERATED}/twim_data.inc ${GENERATED}/twim_funcs.inc ${GENERATED}/scan_funcs.inc
         ${GENERATED}/lifecycle_funcs.inc ${GENERATED}/sensor_loop.inc ${GENERATED}/sensor_wait.inc
         ${GENERATED}/sensor_suspend_heated.inc
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract.py ${TRACKER} ${TWIM} ${GENERATED}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract.py ${TRACKER}/src/sensor/sensor.c
          ${TRACKER}/src/sensor/scan.c ${TWIM}
  VERBATIM)
target_sources(app PRIVATE src/main.c ${GENERATED}/twim_data.inc ${GENERATED}/twim_funcs.inc
               ${GENERATED}/scan_funcs.inc ${GENERATED}/lifecycle_funcs.inc
               ${GENERATED}/sensor_loop.inc ${GENERATED}/sensor_wait.inc ${GENERATED}/sensor_suspend_heated.inc)
target_include_directories(app PRIVATE ${GENERATED} ${TRACKER}/src)
