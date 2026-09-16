#ifndef ONLINE_TEST_SENSOR_H
#define ONLINE_TEST_SENSOR_H
#include <stdbool.h>
bool sensor_fusion_get_mag_ref(float *norm, float *dip);
void sensor_refresh_sensor_ids(void);
#endif
