#ifndef SLIMENRF_CONSOLE
#define SLIMENRF_CONSOLE

#include <stdint.h>

#if CONFIG_USE_SLIMENRF_CONSOLE
int console_serial_start(void);
/* Soft DTR close discards unfinished input/echo, not complete lines accepted
 * into the bounded queue. Reopen preserves their admission, including messages
 * already dequeued. This does not guarantee delivery of unread host bytes. */
void console_serial_close(void);
/* Hard lifecycle loss invalidates queued/dequeued lines; a handler already
 * admitted by the worker is allowed to finish. */
void console_serial_stop(void);
#else
static inline int console_serial_start(void)
{
	return 0;
}

static inline void console_serial_close(void)
{
}

static inline void console_serial_stop(void)
{
}
#endif

#endif
