#ifndef SLIMENRF_CONSOLE
#define SLIMENRF_CONSOLE

void console_thread_create(void);
void console_thread_abort(void);

// Command API for remote execution
void cmd_sens_set(float x, float y, float z);
void cmd_sens_reset(void);
void cmd_reset_zro(void);
void cmd_reset_acc(void);
void cmd_reset_bat(void);
void cmd_reset_tcal(void);
void cmd_fusion_reset(void);
void cmd_ping_start(void);

#endif
