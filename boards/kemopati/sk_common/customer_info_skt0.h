/* SPDX-License-Identifier: Apache-2.0 */
#ifndef SK_CUSTOMER_INFO_SKT0_H
#define SK_CUSTOMER_INFO_SKT0_H

#include "../../../src/system/customer_info_parser.h"

/* Decode SKT0 only; source and identity are left unset, info is zero unless valid. */
void customer_info_skt0_decode(const uint8_t *record, size_t length, struct customer_info_result *out);

#endif
