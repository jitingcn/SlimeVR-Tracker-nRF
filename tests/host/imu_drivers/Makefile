CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra
CFLAGS += -ffunction-sections -fdata-sections -I. -I../../../src -include zephyr/kernel.h
LDFLAGS += -Wl,--gc-sections
LDLIBS += -lm

IMU_SOURCES := \
	../../../src/sensor/imu/BMI270.c \
	../../../src/sensor/imu/ICM42686.c \
	../../../src/sensor/imu/ICM42688.c \
	../../../src/sensor/imu/ICM45686.c

IMU_HEADERS := $(IMU_SOURCES:.c=.h) ../../../src/sensor/imu/BMI270_firmware.h \
	../../../src/sensor/imu/icm426xx_hires.h
COMMON_HEADERS := ../../../src/sensor/sensor.h ../../../src/sensor/sensor_none.h \
	../../../src/sensor/interface.h ../../../src/sensor/diagnostics.h
HOST_HEADERS := $(wildcard zephyr/*.h zephyr/*/*.h hal/*.h)

.PHONY: check clean

check: test_imu_odr
	./test_imu_odr
	python3 run_fifo_temperature.py

test_imu_odr: test_imu_odr.c host_stubs.c $(IMU_SOURCES) $(IMU_HEADERS) $(COMMON_HEADERS) $(HOST_HEADERS) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_imu_odr.c host_stubs.c $(IMU_SOURCES) $(LDLIBS)

clean:
	$(RM) test_imu_odr
