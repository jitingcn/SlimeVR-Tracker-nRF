#ifndef IST83XX_H
#define IST83XX_H

#include "sensor/sensor.h"

struct ist83xx_state {
	uint8_t last_mode_code;
	int64_t oneshot_trigger_ms;
	bool oneshot_pending;
	bool oneshot_failed;
};

int ist83xx_init(struct ist83xx_state *state, bool drive_200, float period_s, float *actual_period_s);
void ist83xx_shutdown(struct ist83xx_state *state);
int ist83xx_update_odr(struct ist83xx_state *state, float period_s, float *actual_period_s);
void ist83xx_mag_oneshot(struct ist83xx_state *state);
bool ist83xx_mag_read(struct ist83xx_state *state, float sensitivity, float m[3]);
void ist83xx_mag_process(uint8_t *raw_m, float sensitivity, float m[3]);

#endif
