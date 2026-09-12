CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra
CFLAGS += -fsanitize=undefined -fno-sanitize-recover=undefined -ffunction-sections -fdata-sections
CFLAGS += -I../imu_drivers -I../../../src -I../../../src/sensor/imu -include ../imu_drivers/zephyr/kernel.h
LDFLAGS += -fsanitize=undefined -Wl,--gc-sections
LDLIBS += -lm
BMI_SOURCE ?= ../../../src/sensor/imu/BMI270.c
HEADERS := $(wildcard ../../../src/sensor/imu/BMI270*.h ../imu_drivers/zephyr/*.h ../imu_drivers/zephyr/*/*.h ../imu_drivers/hal/*.h)

.PHONY: check clean
check: test_bmi_retrim
	./test_bmi_retrim
	./test_bmi_retrim reinit-error

test_bmi_retrim: test_bmi_retrim.c $(BMI_SOURCE) $(HEADERS) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_bmi_retrim.c $(BMI_SOURCE) $(LDLIBS)

clean:
	$(RM) test_bmi_retrim
