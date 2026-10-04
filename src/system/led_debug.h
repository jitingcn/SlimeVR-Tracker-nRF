#ifndef SLIMENRF_LED_DEBUG_H
#define SLIMENRF_LED_DEBUG_H

#ifdef CONFIG_LED_DEBUG
#include <stddef.h>
#include <stdint.h>

/* Existing console tokenizer supplies lowercase tokens. No command changes
 * business state; responses acknowledge only local preview admission. */
void led_debug_command(uint32_t console_session, size_t argc, char **argv);
void led_debug_help(void);
/* Detectable UART/CDC close retires its identity, including parsed late work.
 * RTT uses one live session and the preview's hard deadline. */
void led_debug_session_start(uint32_t console_session);
void led_debug_disconnect(uint32_t console_session);
#endif

#endif
