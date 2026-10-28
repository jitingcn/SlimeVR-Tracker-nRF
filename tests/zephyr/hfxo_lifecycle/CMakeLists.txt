cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_hfxo_lifecycle)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(TRACKER "${CMAKE_CURRENT_SOURCE_DIR}/../../.." CACHE PATH "Tracker checkout")
set(ESB_SOURCE "${TRACKER}/src/connection/esb.c" CACHE FILEPATH "Tracker ESB source (can be a pre-fix snapshot)")
set(NRFX_POWER "$ENV{ZEPHYR_BASE}/../modules/hal/nordic/nrfx/drivers/src/nrfx_power.c" CACHE FILEPATH "SDK nrfx power implementation")
option(HFXO_NRF54 "Exercise nRF54L constant-latency ownership" OFF)
option(HFXO_SYNTH "Exercise LF_SYNTH HFXO retention" OFF)
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_command(
  OUTPUT ${GENERATED}/tracker_clock.inc ${GENERATED}/tracker_pair_gate.inc
         ${GENERATED}/tracker_boot.inc ${GENERATED}/tracker_radio.inc ${GENERATED}/nrfx_power.inc
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract.py ${ESB_SOURCE} ${NRFX_POWER} ${GENERATED}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract.py ${ESB_SOURCE} ${NRFX_POWER}
  VERBATIM)
target_sources(app PRIVATE src/main.c ${GENERATED}/tracker_clock.inc ${GENERATED}/tracker_pair_gate.inc
               ${GENERATED}/tracker_boot.inc ${GENERATED}/tracker_radio.inc ${GENERATED}/nrfx_power.inc)
target_include_directories(app PRIVATE ${GENERATED} ${TRACKER}/src)
target_compile_definitions(app PRIVATE CONFIG_CLOCK_CONTROL_NRF=1)
if(HFXO_NRF54)
  target_compile_definitions(app PRIVATE HFXO_NRF54=1 CONFIG_SOC_SERIES_NRF54L=1)
else()
  target_compile_definitions(app PRIVATE HFXO_NRF54=0)
endif()
if(HFXO_SYNTH)
  target_compile_definitions(app PRIVATE HFXO_SYNTH=1 CONFIG_CLOCK_USE_LF_SYNTH=1)
else()
  target_compile_definitions(app PRIVATE HFXO_SYNTH=0)
endif()
