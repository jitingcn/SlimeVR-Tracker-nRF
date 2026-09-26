# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(board.c)
if(CONFIG_CUSTOMER_INFO AND CONFIG_CUSTOMER_INFO_SKT0)
  zephyr_library_sources(../sk_common/customer_info_skt0.c)
endif()
