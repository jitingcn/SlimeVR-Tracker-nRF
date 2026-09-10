CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -DHOST_SENSOR_DRIVER_TEST -fsanitize=undefined -fno-sanitize-recover=undefined -I. -I../../../src -include zephyr/kernel.h
LDLIBS += -lm -fsanitize=undefined

MAG_SOURCES := \
	../../../src/sensor/mag/AK09940.c \
	../../../src/sensor/mag/BMM150.c \
	../../../src/sensor/mag/BMM350.c \
	../../../src/sensor/mag/IST8306.c \
	../../../src/sensor/mag/IST8308.c \
	../../../src/sensor/mag/LIS2MDL.c \
	../../../src/sensor/mag/LIS3MDL.c \
	../../../src/sensor/mag/MMC5603NJ.c \
	../../../src/sensor/mag/MMC5983MA.c \
	../../../src/sensor/mag/QMC5883L.c \
	../../../src/sensor/mag/QMC6309.c

.PHONY: check clean

check: test_mag_failures test_interface_intervals test_ict153xx test_icm45686 test_lsm6dsv
	./test_mag_failures
	./test_interface_intervals
	./test_ict153xx
	./test_icm45686
	./test_lsm6dsv

test_mag_failures: test_mag_failures.c $(MAG_SOURCES)
	$(CC) $(CFLAGS) -o $@ test_mag_failures.c $(MAG_SOURCES) $(LDLIBS)

test_interface_intervals: test_interface_intervals.c ../../../src/sensor/interface.c
	$(CC) $(CFLAGS) -o $@ test_interface_intervals.c ../../../src/sensor/interface.c $(LDLIBS)

test_ict153xx: test_ict153xx.c ../../../src/sensor/mag/ICT153xx.c
	$(CC) $(CFLAGS) -o $@ test_ict153xx.c ../../../src/sensor/mag/ICT153xx.c $(LDLIBS)

test_icm45686: test_icm45686.c ../../../src/sensor/imu/ICM45686.c ../../../src/sensor/imu/ICM45686.h hal/nrf_gpio.h
	$(CC) $(CFLAGS) -o $@ test_icm45686.c ../../../src/sensor/imu/ICM45686.c $(LDLIBS)

test_lsm6dsv: test_lsm6dsv.c ../../../src/sensor/imu/LSM6DSV.c ../../../src/sensor/imu/LSM6DSV.h hal/nrf_gpio.h
	$(CC) $(CFLAGS) -o $@ test_lsm6dsv.c ../../../src/sensor/imu/LSM6DSV.c $(LDLIBS)

clean:
	$(RM) test_mag_failures test_interface_intervals test_ict153xx test_icm45686 test_lsm6dsv
