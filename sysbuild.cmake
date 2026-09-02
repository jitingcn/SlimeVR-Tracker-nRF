set(soc_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/socs)
set(partition_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/dts/partitions)
set(mcuboot_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/sysbuild)

if(WITH_SOFTDEVICE)
  set_property(TARGET ${DEFAULT_IMAGE} APPEND PROPERTY _EP_CMAKE_ARGS -DWITH_SOFTDEVICE=ON)
endif()

set(uses_uf2_layout FALSE)
if((DEFINED SB_CONFIG_BOARD AND SB_CONFIG_BOARD MATCHES "uf2") OR
   (DEFINED SB_CONFIG_BOARD_QUALIFIERS AND SB_CONFIG_BOARD_QUALIFIERS MATCHES "uf2"))
  set(uses_uf2_layout TRUE)
endif()

set(extra_dtc_overlay_candidates)

if(DEFINED EXTRA_DTC_OVERLAY_FILE)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${EXTRA_DTC_OVERLAY_FILE})
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54LM20A)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf54lm20a_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf54lm20a_mcuboot.overlay)
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54L15)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf54l15_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf54l15_mcuboot.overlay)
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_BOARD STREQUAL "xiao_ble")
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840_xiao_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf52840_xiao_mcuboot.overlay)
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF52840)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf52840_mcuboot.overlay)
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF52833)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf5283x_mcuboot_single.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf5283x_mcuboot_single.overlay)
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF52832)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52832_mcuboot_single.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${partition_overlay_dir}/nrf52832_mcuboot_single.overlay)
elseif(NOT SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54LM20A)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf54lm20a.overlay)
elseif(NOT SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54L15)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf54l15.overlay)
elseif(SB_CONFIG_BOARD STREQUAL "xiao_ble")
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840_xiao.overlay)
elseif(SB_CONFIG_SOC_NRF52840 AND uses_uf2_layout AND WITH_SOFTDEVICE)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840_uf2_sd.overlay)
elseif(SB_CONFIG_SOC_NRF52840 AND uses_uf2_layout)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840_uf2.overlay)
elseif(SB_CONFIG_SOC_NRF52833 AND uses_uf2_layout)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52833_uf2.overlay)
elseif(SB_CONFIG_SOC_NRF52840)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52840.overlay)
elseif(SB_CONFIG_SOC_NRF52832 OR SB_CONFIG_SOC_NRF52833)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52832.overlay)
elseif(SB_CONFIG_SOC_NRF52805 OR SB_CONFIG_SOC_NRF52810 OR SB_CONFIG_SOC_NRF52811)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf52805.overlay)
endif()

if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
  string(REPLACE "/" "_" soc_board_qualifiers "${SB_CONFIG_BOARD_QUALIFIERS}")
  list(APPEND extra_dtc_overlay_candidates ${soc_overlay_dir}/${soc_board_qualifiers}.overlay)

  string(REGEX MATCH "^[^/]+/[^/]+" soc_cpu_qualifiers "${SB_CONFIG_BOARD_QUALIFIERS}")
  if(NOT soc_cpu_qualifiers STREQUAL "")
    string(REPLACE "/" "_" soc_cpu_qualifiers "${soc_cpu_qualifiers}")
    list(APPEND extra_dtc_overlay_candidates ${soc_overlay_dir}/${soc_cpu_qualifiers}.overlay)
  endif()

  string(REGEX REPLACE "/.*$" "" soc_name "${SB_CONFIG_BOARD_QUALIFIERS}")
  list(APPEND extra_dtc_overlay_candidates ${soc_overlay_dir}/${soc_name}.overlay)
endif()

foreach(extra_dtc_overlay_candidate ${extra_dtc_overlay_candidates})
  if(EXISTS ${extra_dtc_overlay_candidate})
    list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${extra_dtc_overlay_candidate})
  endif()
endforeach()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54LM20A)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot_nrf54lm20a.overlay)
  if(SB_CONFIG_BOARD STREQUAL "xiao_nrf54lm20a")
    # XIAO nRF54LM20A: the nRF54 native USB (usbhs) is not wired to the
    # USB-C connector; the onboard SAMD11 serves it as a USB-UART bridge on
    # uart20, so MCUboot serial recovery runs over uart20 instead of USB
    # CDC ACM.
    list(APPEND mcuboot_EXTRA_CONF_FILE
         ${mcuboot_overlay_dir}/mcuboot_nrf54lm20a_uart.conf)
    list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
         ${mcuboot_overlay_dir}/mcuboot_nrf54lm20a_uart.overlay)
  else()
    list(APPEND mcuboot_EXTRA_CONF_FILE
         ${mcuboot_overlay_dir}/mcuboot_usb_next.conf)
  endif()
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF54L15)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot_nrf54l15.overlay)
  if(SB_CONFIG_BOARD STREQUAL "xiao_nrf54l15")
    # XIAO nRF54L15: the nRF54L15 has no native USB controller and the
    # onboard SAMD11 serves the USB-C connector as a USB-UART bridge on
    # uart20, so MCUboot serial recovery runs over uart20 instead of USB
    # CDC ACM.
    list(APPEND mcuboot_EXTRA_CONF_FILE
         ${mcuboot_overlay_dir}/mcuboot_nrf54l15_uart.conf)
    list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
         ${mcuboot_overlay_dir}/mcuboot_nrf54l15_uart.overlay)
  endif()
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND
   (SB_CONFIG_SOC_NRF52833 OR SB_CONFIG_SOC_NRF52840))
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot_usb.overlay)
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND
   (SB_CONFIG_SOC_NRF52833 OR SB_CONFIG_SOC_NRF52840))
  list(APPEND mcuboot_EXTRA_CONF_FILE
       ${mcuboot_overlay_dir}/mcuboot_usb_legacy.conf)
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  set_property(TARGET mcuboot APPEND PROPERTY _EP_CMAKE_ARGS
               -DBOARD_DEFCONFIG:FILEPATH=${mcuboot_overlay_dir}/mcuboot_board_defconfig)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot.overlay
       ${mcuboot_overlay_dir}/mcuboot_boot_mode.overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot_boot_mode.overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_CONF_FILE
       ${CMAKE_CURRENT_LIST_DIR}/boards/mcuboot.conf)
  list(REMOVE_DUPLICATES mcuboot_EXTRA_CONF_FILE)
  set(mcuboot_EXTRA_CONF_FILE
      ${mcuboot_EXTRA_CONF_FILE}
      CACHE INTERNAL "")
  list(REMOVE_DUPLICATES mcuboot_EXTRA_DTC_OVERLAY_FILE)
  set(mcuboot_EXTRA_DTC_OVERLAY_FILE
      ${mcuboot_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()

if(DEFINED ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE)
  list(REMOVE_DUPLICATES ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE)
  set(${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
      ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()

if(DEFINED ${DEFAULT_IMAGE}_EXTRA_CONF_FILE)
  list(REMOVE_DUPLICATES ${DEFAULT_IMAGE}_EXTRA_CONF_FILE)
  set(${DEFAULT_IMAGE}_EXTRA_CONF_FILE
      ${${DEFAULT_IMAGE}_EXTRA_CONF_FILE}
      CACHE INTERNAL "")
endif()
