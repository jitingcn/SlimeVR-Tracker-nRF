set(soc_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/socs)
set(partition_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/dts/partitions)

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

if(SB_CONFIG_SOC_NRF54LM20A)
  list(APPEND extra_dtc_overlay_candidates ${partition_overlay_dir}/nrf54lm20a.overlay)
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

if(DEFINED ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE)
  list(REMOVE_DUPLICATES ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE)
  set(${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
      ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()
