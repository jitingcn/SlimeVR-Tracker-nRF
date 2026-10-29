CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra
CFLAGS += -ffunction-sections -fdata-sections -I. -I../../../src -include zephyr/kernel.h
CFLAGS += -DCONFIG_CONNECTION_RAW_COLLECTION=1 -DCONFIG_SENSOR_USE_MAGNETOMETER=1
LDFLAGS += -Wl,--gc-sections
LDLIBS += -lm

IMU_SOURCES := \
	../../../src/sensor/imu/ICM42686.c \
	../../../src/sensor/imu/ICM42688.c \
	../../../src/sensor/imu/ICM45686.c \
	../../../src/sensor/imu/LSM6DSM.c \
	../../../src/sensor/imu/LSM6DSO.c

IMU_HEADERS := $(IMU_SOURCES:.c=.h) \
	../../../src/sensor/imu/icm426xx_hires.h ../../../src/sensor/imu/icm426xx_io.h
IMU_HEADERS += ../../../src/sensor/imu/icm426xx_odr.h
IMU_HEADERS += ../../../src/sensor/imu/lsm6dsx_select.h
COMMON_HEADERS := ../../../src/sensor/sensor.h ../../../src/sensor/sensor_none.h \
	../../../src/sensor/interface.h ../../../src/sensor/interface_internal.h \
	../../../src/sensor/diagnostics.h ../harness/include/device_types.h \
	../harness/include/i2c_types.h ../harness/include/spi_types.h
HOST_HEADERS := $(wildcard zephyr/*.h zephyr/*/*.h hal/*.h)

.PHONY: check clean

check: test_imu_odr test_imu_io test_lsm6dsm_fifo test_imu_selectors
	./test_imu_odr
	./test_imu_io
	./test_lsm6dsm_fifo
	./test_imu_selectors
	python3 run_fifo_temperature.py

test_imu_odr: test_imu_odr.c host_stubs.c $(IMU_SOURCES) $(IMU_HEADERS) $(COMMON_HEADERS) $(HOST_HEADERS) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_imu_odr.c host_stubs.c $(IMU_SOURCES) $(LDLIBS)

test_imu_io: test_imu_io.c host_stubs.c $(IMU_SOURCES) $(IMU_HEADERS) $(COMMON_HEADERS) $(HOST_HEADERS) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_imu_io.c host_stubs.c $(IMU_SOURCES) $(LDLIBS)

test_lsm6dsm_fifo: test_lsm6dsm_fifo.c host_stubs.c $(IMU_SOURCES) ../../../src/sensor/imu/LSM6DSV.c ../../../src/sensor/imu/LSM6DSV.h $(IMU_HEADERS) $(COMMON_HEADERS) $(HOST_HEADERS) Makefile
	$(CC) $(CFLAGS) -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all $(LDFLAGS) -o $@ test_lsm6dsm_fifo.c host_stubs.c $(IMU_SOURCES) ../../../src/sensor/imu/LSM6DSV.c $(LDLIBS)

test_imu_selectors: test_imu_selectors.c host_stubs.c $(IMU_SOURCES) $(IMU_HEADERS) $(COMMON_HEADERS) $(HOST_HEADERS) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_imu_selectors.c host_stubs.c $(IMU_SOURCES) $(LDLIBS)

clean:
	$(RM) test_imu_odr test_imu_io test_lsm6dsm_fifo test_imu_selectors
