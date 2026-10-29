CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -DHOST_SENSOR_DRIVER_TEST -fsanitize=undefined -fno-sanitize-recover=undefined \
	-ffunction-sections -fdata-sections -I../sensor_drivers -I../../../src -include zephyr/kernel.h
LDFLAGS += -Wl,--gc-sections
LDLIBS += -lm -fsanitize=undefined

DRIVERS := ../../../src/sensor/imu/ICM42686.c ../../../src/sensor/imu/ICM42688.c
HEADERS := ../../../src/sensor/imu/ICM42686.h ../../../src/sensor/imu/ICM42688.h \
	../../../src/sensor/imu/icm426xx_hires.h ../../../src/sensor/imu/icm426xx_io.h \
	../../../src/sensor/imu/icm426xx_odr.h \
	../../../src/sensor/sensor.h \
	../../../src/sensor/sensor_none.h ../../../src/sensor/interface.h \
	../../../src/sensor/diagnostics.h \
	../sensor_drivers/zephyr/kernel.h ../sensor_drivers/zephyr/logging/log.h \
	../sensor_drivers/zephyr/drivers/i2c.h ../sensor_drivers/zephyr/drivers/spi.h \
	../sensor_drivers/hal/nrf_gpio.h ../harness/include/device_types.h \
	../harness/include/i2c_types.h ../harness/include/spi_types.h

.PHONY: check clean

check: test_icm426xx_decode
	./test_icm426xx_decode

test_icm426xx_decode: test_icm426xx_decode.c ../sensor_drivers/host_log.c $(DRIVERS) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_icm426xx_decode.c ../sensor_drivers/host_log.c $(DRIVERS) $(LDLIBS)

clean:
	$(RM) test_icm426xx_decode
