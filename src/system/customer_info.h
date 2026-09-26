#ifndef CUSTOMER_INFO_H
#define CUSTOMER_INFO_H

#include <stdbool.h>
#include "customer_info_parser.h"

/* Reader convention only: this does not reserve or claim CUSTOMER ownership. */
#define CUSTOMER_INFO_CUSTOMER_OFFSET 64U

void customer_info_read(struct customer_info_result *out);
#if CONFIG_CUSTOMER_INFO && CONFIG_CUSTOMER_INFO_SKT0
/* Board-owned compatibility reader, called only when the SNCI slot B is erased.
 * Fills the complete result, including source and board-specific identity.
 * An erased board record remains absent with no source or format.
 */
void customer_info_read_board_variant(struct customer_info_result *out);
#endif
enum customer_info_report_mode {
	CUSTOMER_INFO_REPORT_LOG_SUMMARY,
	CUSTOMER_INFO_REPORT_CONSOLE_SUMMARY,
	CUSTOMER_INFO_REPORT_CONSOLE_DETAILS,
};

void customer_info_report(enum customer_info_report_mode mode);

#endif
