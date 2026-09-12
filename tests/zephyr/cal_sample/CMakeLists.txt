cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_cal_sample)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(CAL_SAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/sensor/calibration/cal_sample.c" CACHE FILEPATH "Calibration sample source under test")
set(GENERATED_SAMPLE "${CMAKE_CURRENT_BINARY_DIR}/cal_sample.inc")
add_custom_command(
  OUTPUT ${GENERATED_SAMPLE}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sample.py ${CAL_SAMPLE_SOURCE} ${GENERATED_SAMPLE}
  DEPENDS ${CAL_SAMPLE_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_sample.py
  VERBATIM
)
target_sources(app PRIVATE src/main.c ${GENERATED_SAMPLE})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ../../../src)
