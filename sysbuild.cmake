set(pm_static_dir ${CMAKE_CURRENT_LIST_DIR}/pm_static)
set(soc_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/socs)

set(pm_static_override_candidates)

if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
  string(REPLACE "/" "_" pm_static_board_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
  list(APPEND pm_static_override_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}_${pm_static_board_qualifiers}.yml
    ${pm_static_dir}/${SB_CONFIG_BOARD}_${pm_static_board_qualifiers}.yml
  )
endif()

list(APPEND pm_static_override_candidates
  ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}.yml
  ${pm_static_dir}/${SB_CONFIG_BOARD}.yml
)

foreach(pm_static_override_candidate ${pm_static_override_candidates})
  if(EXISTS ${pm_static_override_candidate})
    set(PM_STATIC_YML_FILE ${pm_static_override_candidate} CACHE INTERNAL "")
    break()
  endif()
endforeach()

if(NOT DEFINED PM_STATIC_YML_FILE)
  if(SB_CONFIG_BOARD STREQUAL "xiao_ble")
    set(PM_STATIC_YML_FILE ${pm_static_dir}/nrf52840_xiao.yml CACHE INTERNAL "")
  elseif(SB_CONFIG_BOARD MATCHES "^nrf52840dongle$")
    set(PM_STATIC_YML_FILE ${pm_static_dir}/nrf52840dongle.yml CACHE INTERNAL "")
  elseif(SB_CONFIG_SOC_NRF52833)
    set(PM_STATIC_YML_FILE ${pm_static_dir}/nrf52833_uf2.yml CACHE INTERNAL "")
  elseif(SB_CONFIG_SOC_NRF52840)
    set(PM_STATIC_YML_FILE ${pm_static_dir}/nrf52840_uf2.yml CACHE INTERNAL "")
  endif()
endif()

if(NOT DEFINED ${DEFAULT_IMAGE}_DTC_OVERLAY_FILE)
  set(dtc_overlay_candidates)

  if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
    string(REPLACE "/" "_" soc_board_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
    list(APPEND dtc_overlay_candidates ${soc_overlay_dir}/${soc_board_qualifiers}.overlay)

    string(REGEX REPLACE "/.*$" "" soc_name ${SB_CONFIG_BOARD_QUALIFIERS})
    list(APPEND dtc_overlay_candidates ${soc_overlay_dir}/${soc_name}.overlay)
  endif()

  foreach(dtc_overlay_candidate ${dtc_overlay_candidates})
    if(EXISTS ${dtc_overlay_candidate})
      set(${DEFAULT_IMAGE}_DTC_OVERLAY_FILE ${dtc_overlay_candidate} CACHE INTERNAL "")
      break()
    endif()
  endforeach()
endif()
