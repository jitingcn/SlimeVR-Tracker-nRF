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

check: test_mag_failures test_interface_intervals
	./test_mag_failures
	./test_interface_intervals

test_mag_failures: test_mag_failures.c $(MAG_SOURCES)
	$(CC) $(CFLAGS) -o $@ test_mag_failures.c $(MAG_SOURCES) $(LDLIBS)

test_interface_intervals: test_interface_intervals.c ../../../src/sensor/interface.c
	$(CC) $(CFLAGS) -o $@ test_interface_intervals.c ../../../src/sensor/interface.c $(LDLIBS)

clean:
	$(RM) test_mag_failures test_interface_intervals
