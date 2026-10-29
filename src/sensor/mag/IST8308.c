#include "IST8308.h"
#include "ist83xx.h"
#include "sensor/sensor_none.h"

static struct ist83xx_state state = {.last_mode_code = 0xff};

int ist8308_init(float period_s, float *actual_period_s)
{
	return ist83xx_init(&state, true, period_s, actual_period_s);
}

void ist8308_shutdown(void)
{
	ist83xx_shutdown(&state);
}

int ist8308_update_odr(float period_s, float *actual_period_s)
{
	return ist83xx_update_odr(&state, period_s, actual_period_s);
}

void ist8308_mag_oneshot(void)
{
	ist83xx_mag_oneshot(&state);
}

bool ist8308_mag_read(float m[3])
{
	return ist83xx_mag_read(&state, 0.075f, m);
}

void ist8308_mag_process(uint8_t *raw_m, float m[3])
{
	ist83xx_mag_process(raw_m, 0.075f, m);
}

const sensor_mag_t sensor_mag_ist8308 = {
	ist8308_init,
	ist8308_shutdown,
	ist8308_update_odr,
	ist8308_mag_oneshot,
	ist8308_mag_read,
	mag_none_temp_read,
	ist8308_mag_process,
	7,
	7,
};
