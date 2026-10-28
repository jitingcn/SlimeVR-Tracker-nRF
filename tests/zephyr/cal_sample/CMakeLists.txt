cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_cal_sample)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CAL_SAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/sensor/calibration/cal_sample.c" CACHE FILEPATH "Calibration sample source under test")
get_filename_component(CAL_SAMPLE_SOURCE_DIR "${CAL_SAMPLE_SOURCE}" DIRECTORY)
get_filename_component(TRACKER_SOURCE_ROOT "${CAL_SAMPLE_SOURCE_DIR}/../../.." ABSOLUTE)
set(GENERATED_SAMPLE "${CMAKE_CURRENT_BINARY_DIR}/cal_sample.inc")
add_custom_command(
  OUTPUT ${GENERATED_SAMPLE}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sample.py ${CAL_SAMPLE_SOURCE} ${GENERATED_SAMPLE}
  DEPENDS ${CAL_SAMPLE_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sample.py
    ${TRACKER_SOURCE_ROOT}/src/sensor/sensor.c
    ${TRACKER_SOURCE_ROOT}/src/sensor/calibration/cal_sens.c
    ${TRACKER_SOURCE_ROOT}/src/sensor/calibration/bias_collect.c
    ${TRACKER_SOURCE_ROOT}/src/sensor/imu/LSM6DSV.c
    ${TRACKER_SOURCE_ROOT}/src/sensor/imu/LSM6DSO.c
    ${TRACKER_SOURCE_ROOT}/src/util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../host/harness/python/c_extract.py
  VERBATIM
)
target_sources(app PRIVATE src/main.c ${GENERATED_SAMPLE})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ../../../src)
target_compile_definitions(app PRIVATE CONFIG_CONNECTION_RAW_COLLECTION=1 CONFIG_SENSOR_USE_MAGNETOMETER=1 CONFIG_SENSOR_USE_ACCEL_CALIBRATION=1)
