cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tracker_hid_lifecycle)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_filename_component(TRACKER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(TRACKER_SOURCE_ROOT "${TRACKER_ROOT}" CACHE PATH "Tracker source under test")
set(GENERATED_HID "${CMAKE_CURRENT_BINARY_DIR}/hid_lifecycle.inc")
set(GENERATED_USB "${CMAKE_CURRENT_BINARY_DIR}/usb_lifecycle.inc")
set(GENERATED_UDC "${CMAKE_CURRENT_BINARY_DIR}/udc_enqueue.inc")
add_custom_command(
  OUTPUT ${GENERATED_HID} ${GENERATED_USB} ${GENERATED_UDC}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract.py
          ${TRACKER_SOURCE_ROOT} ${ZEPHYR_BASE} ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${TRACKER_SOURCE_ROOT}/src/hid.c ${TRACKER_SOURCE_ROOT}/src/usb.c
          ${ZEPHYR_BASE}/drivers/usb/udc/udc_common.h
          ${ZEPHYR_BASE}/drivers/usb/udc/udc_common.c
          ${ZEPHYR_BASE}/drivers/usb/udc/udc_nrf.c
          ${CMAKE_CURRENT_SOURCE_DIR}/extract.py
  VERBATIM
)
target_sources(app PRIVATE src/main.c ${GENERATED_HID} ${GENERATED_USB} ${GENERATED_UDC})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${TRACKER_SOURCE_ROOT}/src)
