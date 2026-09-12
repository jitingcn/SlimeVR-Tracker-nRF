CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -fsanitize=undefined -fno-sanitize-recover=undefined -pthread -I. -I.generated -I../../../src
CFLAGS += -DCONFIG_SENSOR_USE_6_SIDE_CALIBRATION=1 -DCONFIG_SENSOR_USE_TCAL=1 -DCONFIG_CMSIS_DSP=0 -DCONFIG_SENSOR_USE_SENS_CALIBRATION=0
LDLIBS += -lm -fsanitize=undefined -pthread

SOURCES := test_imu_calibration.c ../../../src/sensor/calibration/imu_calibration.c
HEADERS := ../../../src/sensor/calibration/imu_calibration.h ../../../src/sensor/calibration/calibration.h globals.h system/system.h util.h zephyr/kernel.h

.PHONY: check clean

check: test_imu_calibration test_lifecycle test_button test_button_extra
	./test_imu_calibration
	./test_lifecycle
	./test_button
	./test_button_extra

test_imu_calibration: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(SOURCES) $(LDLIBS)

.generated.stamp: extract_lifecycle.py ../../../src/system/system.c ../../../src/sensor/sensor.c
	python3 extract_lifecycle.py .generated
	touch $@

test_lifecycle: test_lifecycle.c ../../../src/sensor/calibration/imu_calibration.c $(HEADERS) .generated.stamp
	$(CC) $(CFLAGS) -DTEST_LIFECYCLE -o $@ test_lifecycle.c ../../../src/sensor/calibration/imu_calibration.c $(LDLIBS)

test_button: test_button.c .generated.stamp
	$(CC) $(CFLAGS) -DCONFIG_USER_EXTRA_ACTIONS=0 -o $@ test_button.c $(LDLIBS)

test_button_extra: test_button.c .generated.stamp
	$(CC) $(CFLAGS) -DCONFIG_USER_EXTRA_ACTIONS=1 -o $@ test_button.c $(LDLIBS)

clean:
	$(RM) test_imu_calibration test_lifecycle test_button test_button_extra .generated.stamp
	$(RM) -r .generated
