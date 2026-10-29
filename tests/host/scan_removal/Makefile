CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -Wno-unused-parameter -fsanitize=address,undefined -fno-sanitize-recover=all
CPPFLAGS += -I. -I../imu_drivers -I../../../src -include zephyr/kernel.h
CPPFLAGS += -DCONFIG_SENSOR_USE_MAGNETOMETER=0
LDFLAGS += -fsanitize=address,undefined

SOURCES := test_scan_removal.c ../../../src/sensor/scan.c ../../../src/sensor/scan_spi.c \
	../../../src/sensor/sensors_tables.c ../../../src/sensor/sensor_none.c
HEADERS := $(wildcard zephyr/*.h zephyr/*/*.h) app_version.h \
	../imu_drivers/zephyr/kernel.h ../imu_drivers/zephyr/types.h \
	../imu_drivers/zephyr/logging/log.h ../harness/include/driver_util.h \
	../harness/include/device_types.h ../harness/include/i2c_types.h ../harness/include/spi_types.h \
	$(wildcard ../../../src/sensor/*.h) \
	$(wildcard ../../../src/sensor/imu/*.h) ../../../src/build_defines.h \
	../../../src/system/status.h

FULL_CONFIG := -DCONFIG_SENSOR_DRV_ICM42688=1 -DCONFIG_SENSOR_DRV_ICM42686=1 \
	-DCONFIG_SENSOR_DRV_ICM45686=1 -DCONFIG_SENSOR_DRV_LSM6DSM=1 \
	-DCONFIG_SENSOR_DRV_LSM6DSO=1 -DCONFIG_SENSOR_DRV_LSM6DSV=1
MINIMAL_CONFIG := -DCONFIG_SENSOR_IMU_DRIVERS_MINIMAL=1
MIXED_CONFIG := $(MINIMAL_CONFIG) -DCONFIG_SENSOR_DRV_ICM42688=1 \
	-DCONFIG_SENSOR_DRV_ICM45686=1 -DCONFIG_SENSOR_DRV_LSM6DSV=1
GROUP_A_CONFIG := $(MINIMAL_CONFIG) -DCONFIG_SENSOR_DRV_ICM42686=1
GROUP_B_CONFIG := $(MINIMAL_CONFIG) -DCONFIG_SENSOR_DRV_LSM6DSO=1

PROBE_CPPFLAGS := -I../sensor_drivers -I../imu_drivers -I../../../src \
	-include ../sensor_drivers/zephyr/kernel.h -DCONFIG_SENSOR_USE_MAGNETOMETER=1 \
	-DCONFIG_SENSOR_DRV_ICT153XX=1 -DCONFIG_CONNECTION_RAW_COLLECTION=1
PROBE_SOURCES := test_ext_probes.c ../sensor_drivers/host_log.c \
	../../../src/sensor/scan_ext.c ../../../src/sensor/interface.c
PROBE_HEADERS := $(wildcard ../../../src/sensor/*.h ../../../src/sensor/imu/*.h) \
	../../../src/sensor/mag/ICT153xx.h \
	$(wildcard ../sensor_drivers/zephyr/*.h ../sensor_drivers/zephyr/*/*.h) \
	$(wildcard ../harness/include/*.h)

.PHONY: check clean

check: test_full test_minimal_mixed test_minimal_group_a test_minimal_group_b \
	test_ext_dso test_ext_dsv test_ext_icm
	./test_full
	./test_minimal_mixed
	./test_minimal_group_a
	./test_minimal_group_b
	./test_ext_dso
	./test_ext_dsv
	./test_ext_icm

test_full: $(SOURCES) $(HEADERS) Makefile
	$(CC) $(CPPFLAGS) $(FULL_CONFIG) $(CFLAGS) $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test_minimal_mixed: $(SOURCES) $(HEADERS) Makefile
	$(CC) $(CPPFLAGS) $(MIXED_CONFIG) $(CFLAGS) $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test_minimal_group_a: $(SOURCES) $(HEADERS) Makefile
	$(CC) $(CPPFLAGS) $(GROUP_A_CONFIG) $(CFLAGS) $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test_minimal_group_b: $(SOURCES) $(HEADERS) Makefile
	$(CC) $(CPPFLAGS) $(GROUP_B_CONFIG) $(CFLAGS) $(SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test_ext_dso: $(PROBE_SOURCES) $(PROBE_HEADERS) ../sensor_drivers/test_lsm6dso.c \
		../../../src/sensor/imu/LSM6DSO.c ../../../src/sensor/imu/LSM6DSV.c \
		../../../src/sensor/mag/QMC6309.c ../../../src/sensor/sensor_none.c Makefile
	$(CC) $(PROBE_CPPFLAGS) -DPROBE_DSO=1 $(CFLAGS) -ffunction-sections -fdata-sections \
		$(PROBE_SOURCES) ../../../src/sensor/imu/LSM6DSO.c ../../../src/sensor/imu/LSM6DSV.c \
		../../../src/sensor/mag/QMC6309.c ../../../src/sensor/sensor_none.c \
		$(LDFLAGS) -Wl,--gc-sections -lm -o $@

test_ext_dsv: $(PROBE_SOURCES) $(PROBE_HEADERS) ../sensor_drivers/test_lsm6dsv.c \
		../../../src/sensor/imu/LSM6DSV.c Makefile
	$(CC) $(PROBE_CPPFLAGS) -DPROBE_DSV=1 $(CFLAGS) -ffunction-sections -fdata-sections \
		$(PROBE_SOURCES) ../../../src/sensor/imu/LSM6DSV.c $(LDFLAGS) -Wl,--gc-sections -lm -o $@

test_ext_icm: $(PROBE_SOURCES) $(PROBE_HEADERS) ../sensor_drivers/test_icm45686.c \
		../../../src/sensor/imu/ICM45686.c Makefile
	$(CC) $(PROBE_CPPFLAGS) -DPROBE_ICM=1 $(CFLAGS) -ffunction-sections -fdata-sections \
		$(PROBE_SOURCES) ../../../src/sensor/imu/ICM45686.c $(LDFLAGS) -Wl,--gc-sections -lm -o $@

clean:
	$(RM) test_full test_minimal_mixed test_minimal_group_a test_minimal_group_b \
		test_ext_dso test_ext_dsv test_ext_icm
