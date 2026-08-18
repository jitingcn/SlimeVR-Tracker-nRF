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

.PHONY: check clean

check: test_imu_odr
	./test_imu_odr

test_imu_odr: test_imu_odr.c host_stubs.c $(IMU_SOURCES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_imu_odr.c host_stubs.c $(IMU_SOURCES) $(LDLIBS)

clean:
	$(RM) test_imu_odr
