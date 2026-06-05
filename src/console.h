#ifndef SLIMENRF_CONSOLE
#define SLIMENRF_CONSOLE

#include <stdint.h>

void console_thread_create(void);
void console_thread_abort(void);

// Command API for remote execution
void cmd_sens_set(float x, float y, float z);
void cmd_sens_auto(const char *axis_str, const char *rev_str);
void cmd_sens_auto_request(uint8_t axis, uint16_t revolutions);
void cmd_sens_reset(void);
void cmd_reset_zro(void);
void cmd_reset_acc(void);
void cmd_reset_bat(void);
void cmd_reset_tcal(void);
void cmd_fusion_reset(void);
void cmd_ping_start(void);
void cmd_shutdown(void);

#endif
