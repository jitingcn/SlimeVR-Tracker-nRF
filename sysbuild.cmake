set(pm_static_dir ${CMAKE_CURRENT_LIST_DIR}/pm_static)
set(soc_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/socs)
set(app_config_dir ${CMAKE_CURRENT_LIST_DIR})

set(pm_static_candidates)

if(DEFINED FILE_SUFFIX AND NOT FILE_SUFFIX STREQUAL "")
  # (pm_static) + BOARD + QUALIFIERS
  if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
    string(REPLACE "/" "_" pm_static_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
    list(APPEND pm_static_candidates
      ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}_${pm_static_qualifiers}_${FILE_SUFFIX}.yml
      ${pm_static_dir}/${SB_CONFIG_BOARD}_${pm_static_qualifiers}_${FILE_SUFFIX}.yml
    )
  endif()

  # (pm_static) + BOARD
  list(APPEND pm_static_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}_${FILE_SUFFIX}.yml
    ${pm_static_dir}/${SB_CONFIG_BOARD}_${FILE_SUFFIX}.yml
  )

  # (pm_static) + SOC + "uf2" in BOARD or QUALIFIERS
  if((DEFINED SB_CONFIG_BOARD AND SB_CONFIG_BOARD MATCHES "uf2") OR (DEFINED SB_CONFIG_BOARD_QUALIFIERS AND SB_CONFIG_BOARD_QUALIFIERS MATCHES "uf2"))
    list(APPEND pm_static_candidates
      ${pm_static_dir}/pm_static_${SB_CONFIG_SOC}_uf2_${FILE_SUFFIX}.yml
      ${pm_static_dir}/${SB_CONFIG_SOC}_uf2_${FILE_SUFFIX}.yml
    )
  endif()
endif()

# (pm_static) + BOARD + QUALIFIERS
if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
  string(REPLACE "/" "_" pm_static_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
  list(APPEND pm_static_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}_${pm_static_qualifiers}.yml
    ${pm_static_dir}/${SB_CONFIG_BOARD}_${pm_static_qualifiers}.yml
  )
endif()

# (pm_static) + BOARD
list(APPEND pm_static_candidates
  ${pm_static_dir}/pm_static_${SB_CONFIG_BOARD}.yml
  ${pm_static_dir}/${SB_CONFIG_BOARD}.yml
)

# (pm_static) + SOC + "uf2" in BOARD or QUALIFIERS
if((DEFINED SB_CONFIG_BOARD AND SB_CONFIG_BOARD MATCHES "uf2") OR (DEFINED SB_CONFIG_BOARD_QUALIFIERS AND SB_CONFIG_BOARD_QUALIFIERS MATCHES "uf2"))
  list(APPEND pm_static_candidates
    ${pm_static_dir}/pm_static_${SB_CONFIG_SOC}_uf2.yml
    ${pm_static_dir}/${SB_CONFIG_SOC}_uf2.yml
  )
endif()

list(REMOVE_DUPLICATES pm_static_candidates)

#message("-- Partition manager static configuration search candidates: ${pm_static_candidates}")

foreach(pm_static_candidate ${pm_static_candidates})
  if(EXISTS ${pm_static_candidate})
    set(PM_STATIC_YML_FILE ${pm_static_candidate} CACHE INTERNAL "")
    break()
  endif()
endforeach()

if(DEFINED FILE_SUFFIX AND NOT FILE_SUFFIX STREQUAL "")
  set(app_extra_conf_candidate ${app_config_dir}/prj_${FILE_SUFFIX}.conf)

  if(EXISTS ${app_extra_conf_candidate})
    list(APPEND ${DEFAULT_IMAGE}_EXTRA_CONF_FILE ${app_extra_conf_candidate})
    list(REMOVE_DUPLICATES ${DEFAULT_IMAGE}_EXTRA_CONF_FILE)
    set(${DEFAULT_IMAGE}_EXTRA_CONF_FILE
        ${${DEFAULT_IMAGE}_EXTRA_CONF_FILE}
        CACHE INTERNAL "")
  endif()
endif()

set(extra_dtc_overlay_candidates)

if(DEFINED SB_CONFIG_BOARD_QUALIFIERS AND NOT SB_CONFIG_BOARD_QUALIFIERS STREQUAL "")
  string(REPLACE "/" "_" soc_board_qualifiers ${SB_CONFIG_BOARD_QUALIFIERS})
  list(APPEND extra_dtc_overlay_candidates ${soc_overlay_dir}/${soc_board_qualifiers}.overlay)

  string(REGEX REPLACE "/.*$" "" soc_name ${SB_CONFIG_BOARD_QUALIFIERS})
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
