#include <SEGGER_RTT.h>
#include <stdio.h>
#include <stdbool.h>

static char line[64];

/* Never return a truncated command prefix. Consume the entire physical line
 * and let the worker invalidate any staged transaction before dispatch. */
static char *rtt_console_getline(bool *overflow)
{
	static bool last_was_cr;
	size_t len = 0;
	*overflow = false;
#if CONFIG_SENSOR_USE_TCAL
	bool capture_active = sensor_tcal_backup_active();
#endif
	for (;;) {
		while (SEGGER_RTT_HasKey() == 0) {
#if CONFIG_SENSOR_USE_TCAL
			if (capture_active) {
				sensor_tcal_backup_process();
				capture_active = sensor_tcal_backup_active();
				k_msleep(10);
				continue;
			}
#endif
			k_usleep(1);
		}
		int byte = SEGGER_RTT_GetKey();
		if (byte < 0) {
			continue;
		}
		if (byte == '\n' && last_was_cr) {
			last_was_cr = false;
			continue;
		}
		last_was_cr = byte == '\r';
#if CONFIG_SENSOR_USE_TCAL
		int capture = sensor_tcal_backup_input_byte((uint8_t)byte);
		if (capture != 0) {
			if (capture == 2) {
				line[0] = '\0';
				return line;
			}
			continue;
		}
#endif
		printk("%c", byte);
		if (byte == '\n' || byte == '\r') {
			line[len] = '\0';
			return line;
		}
		if (len < sizeof(line) - 1) {
			line[len++] = (char)byte;
		} else {
			*overflow = true;
		}
	}
}
