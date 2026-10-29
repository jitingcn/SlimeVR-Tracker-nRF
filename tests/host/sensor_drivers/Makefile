CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -DHOST_SENSOR_DRIVER_TEST -fsanitize=undefined -fno-sanitize-recover=undefined -I. -I../../../src -include zephyr/kernel.h
CFLAGS += -DCONFIG_CONNECTION_RAW_COLLECTION=1 -DCONFIG_SENSOR_USE_MAGNETOMETER=1
LDLIBS += -lm -fsanitize=undefined

MAG_SOURCES := \
	../../../src/sensor/mag/AK09940.c \
	../../../src/sensor/mag/BMM150.c \
	../../../src/sensor/mag/BMM350.c \
	../../../src/sensor/mag/ist83xx.c \
	../../../src/sensor/mag/IST8306.c \
	../../../src/sensor/mag/IST8308.c \
	../../../src/sensor/mag/LIS2MDL.c \
	../../../src/sensor/mag/LIS3MDL.c \
	../../../src/sensor/mag/MMC5603NJ.c \
	../../../src/sensor/mag/MMC5983MA.c \
	../../../src/sensor/mag/QMC5883L.c \
	../../../src/sensor/mag/QMC6309.c

COMMON_HEADERS := ../../../src/sensor/sensor.h ../../../src/sensor/sensor_none.h \
	../../../src/sensor/interface.h ../../../src/sensor/interface_internal.h ../../../src/sensor/diagnostics.h
HOST_HEADERS := $(wildcard zephyr/*.h zephyr/*/*.h hal/*.h)
HOST_LOG := host_log.c

.PHONY: check clean

check: test_mag_failures test_ist8306_only test_ist8308_only test_interface_intervals test_ict153xx test_ict_identity test_icm45686 test_lsm6dsv test_lsm6dso
	./test_mag_failures
	./test_ist8306_only
	./test_ist8308_only
	./test_interface_intervals
	./test_ict153xx
	./test_ict_identity
	./test_icm45686
	./test_lsm6dsv
	./test_lsm6dso

test_mag_failures: test_mag_failures.c $(MAG_SOURCES) $(MAG_SOURCES:.c=.h) $(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -o $@ test_mag_failures.c $(MAG_SOURCES) $(HOST_LOG) $(LDLIBS)

test_ist8306_only: test_mag_failures.c ../../../src/sensor/mag/ist83xx.c ../../../src/sensor/mag/ist83xx.h ../../../src/sensor/mag/IST8306.c $(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -DTEST_IST8306_ONLY -Wno-unused-function -ffunction-sections -fdata-sections -Wl,--gc-sections -o $@ test_mag_failures.c ../../../src/sensor/mag/ist83xx.c ../../../src/sensor/mag/IST8306.c $(HOST_LOG) $(LDLIBS)

test_ist8308_only: test_mag_failures.c ../../../src/sensor/mag/ist83xx.c ../../../src/sensor/mag/ist83xx.h ../../../src/sensor/mag/IST8308.c $(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -DTEST_IST8308_ONLY -Wno-unused-function -ffunction-sections -fdata-sections -Wl,--gc-sections -o $@ test_mag_failures.c ../../../src/sensor/mag/ist83xx.c ../../../src/sensor/mag/IST8308.c $(HOST_LOG) $(LDLIBS)

test_interface_intervals: test_interface_intervals.c ../../../src/sensor/interface.c \
		../../../src/sensor/interface.h ../../../src/sensor/interface_internal.h $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -o $@ test_interface_intervals.c ../../../src/sensor/interface.c $(HOST_LOG) $(LDLIBS)

test_ict153xx: test_ict153xx.c ../../../src/sensor/mag/ICT153xx.c ../../../src/sensor/mag/ICT153xx.h \
		$(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -o $@ test_ict153xx.c ../../../src/sensor/mag/ICT153xx.c $(HOST_LOG) $(LDLIBS)

test_ict_identity: test_ict_identity.c ../../../src/sensor/mag/ICT153xx.c ../../../src/sensor/interface.c \
		$(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -o $@ test_ict_identity.c ../../../src/sensor/mag/ICT153xx.c \
		../../../src/sensor/interface.c $(HOST_LOG) $(LDLIBS)

test_icm45686: test_icm45686.c ../../../src/sensor/imu/ICM45686.c ../../../src/sensor/imu/ICM45686.h \
		../../../src/sensor/imu/icm426xx_hires.h \
		$(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) model_ssi_io.c Makefile
	$(CC) $(CFLAGS) -o $@ test_icm45686.c ../../../src/sensor/imu/ICM45686.c model_ssi_io.c $(HOST_LOG) $(LDLIBS)

test_lsm6dsv: test_lsm6dsv.c ../../../src/sensor/imu/LSM6DSV.c ../../../src/sensor/imu/LSM6DSV.h \
		$(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) model_ssi_io.c Makefile
	$(CC) $(CFLAGS) -o $@ test_lsm6dsv.c ../../../src/sensor/imu/LSM6DSV.c model_ssi_io.c $(HOST_LOG) $(LDLIBS)

test_lsm6dso: test_lsm6dso.c ../../../src/sensor/imu/LSM6DSO.c ../../../src/sensor/interface.c \
		../../../src/sensor/mag/QMC6309.c ../../../src/sensor/imu/LSM6DSO.h \
		../../../src/sensor/imu/LSM6DSV.h ../../../src/sensor/mag/QMC6309.h \
		../../../src/sensor/imu/lsm6dsx_select.h \
		$(COMMON_HEADERS) $(HOST_HEADERS) $(HOST_LOG) Makefile
	$(CC) $(CFLAGS) -ffunction-sections -fdata-sections -Wl,--gc-sections -o $@ \
		test_lsm6dso.c ../../../src/sensor/imu/LSM6DSO.c ../../../src/sensor/interface.c \
		../../../src/sensor/mag/QMC6309.c $(HOST_LOG) $(LDLIBS)

clean:
	$(RM) test_mag_failures test_ist8306_only test_ist8308_only test_interface_intervals test_ict153xx test_ict_identity test_icm45686 test_lsm6dsv test_lsm6dso
