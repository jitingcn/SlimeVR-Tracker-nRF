#ifndef SLIMENRF_CONSOLE
#define SLIMENRF_CONSOLE

#include <stdint.h>

#if CONFIG_USE_SLIMENRF_CONSOLE
int console_serial_start(void);
void console_serial_stop(void);
#else
static inline int console_serial_start(void)
{
	return 0;
}

static inline void console_serial_stop(void)
{
}
#endif


#endif
