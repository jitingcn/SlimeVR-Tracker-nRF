cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_sensor_recovery)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(SENSOR_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/sensor/sensor.c" CACHE FILEPATH "Sensor lifecycle source under test")
set(GENERATED_SENSOR "${CMAKE_CURRENT_BINARY_DIR}/sensor_lifecycle.inc")
add_custom_command(
  OUTPUT ${GENERATED_SENSOR}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sensor.py ${SENSOR_SOURCE} ${GENERATED_SENSOR}
  DEPENDS ${SENSOR_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sensor.py
  VERBATIM
)
target_sources(app PRIVATE src/main.c ${GENERATED_SENSOR})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ../../../src)
