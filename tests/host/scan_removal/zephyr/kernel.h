#ifndef HOST_SCAN_REMOVAL_KERNEL_H
#define HOST_SCAN_REMOVAL_KERNEL_H

#include <stdbool.h>
#include "../../imu_drivers/zephyr/kernel.h"

/* Match Zephyr's IS_ENABLED for options which are absent as well as 0 or 1. */
#define HOST_CONFIG_PLACEHOLDER_1 0,
#define HOST_CONFIG_SECOND(_ignored, value, ...) value
#define HOST_CONFIG_ENABLED_1(config) HOST_CONFIG_ENABLED_2(HOST_CONFIG_PLACEHOLDER_##config)
#define HOST_CONFIG_ENABLED_2(value) HOST_CONFIG_SECOND(value 1, 0)
#define IS_ENABLED(config) HOST_CONFIG_ENABLED_1(config)

#endif
