#include "connection/relay_protocol.h"
#include <stdio.h>
int main(void)
{
	for (unsigned width = 15; width <= 16; width++) {
		unsigned callback = relay_second_b_offset(10, 4, width, 1) - RELAY_V3_TX_OFFSET - 3;
		printf(
			"adjacent width=%u callback=%u first_END=%u slack=%u ticks\n",
			width,
			callback,
			RELAY_V3_CONTROL_AIR_TICKS,
			callback - RELAY_V3_CONTROL_AIR_TICKS
		);
		if (callback < RELAY_V3_CONTROL_AIR_TICKS + 8U) {
			return 1;
		}
	}
	return 0;
}
