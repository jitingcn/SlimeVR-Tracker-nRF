CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
FLAGS = $(CFLAGS) -Wno-unused-variable -Wno-unused-parameter -Wno-error=cpp -I.
BINS = test_sync_pwm test_sync_strip test_sync_strip_p10 test_sync_strip_ungated test_sync_gpio test_local_pwm test_sync_nrf54 test_noled
DEPS = test_led_sync.c globals.h zephyr/kernel.h ../../../src/system/led.c ../../../src/system/led.h ../../../src/system/led_sync.h ../../../src/system/led_strip_fade.h
.PHONY: check clean
check: $(BINS)
	@set -e; for test in $(BINS); do ./$$test; done

test_sync_pwm: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=0 -o $@ $<
test_sync_strip: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=1 -o $@ $<
test_sync_strip_p10: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=1 \
		-DHOST_P10=1 -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=1000 \
		-DCONFIG_LED_DEFAULT_COLOR_R=10000 -DCONFIG_LED_DEFAULT_COLOR_G=3137 \
		-DCONFIG_LED_DEFAULT_COLOR_B=5294 -o $@ $<
test_sync_strip_ungated: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=1 -DHOST_GATE=0 -o $@ $<
test_sync_gpio: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=0 -DHOST_PWM=0 -o $@ $<
test_local_pwm: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=0 -DCONFIG_LED_STRIP=0 -o $@ $<
test_sync_nrf54: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=0 -DCONFIG_SYS_CLOCK_TICKS_PER_SEC=31250 -o $@ $<
test_noled: test_noled.c globals.h ../../../src/system/led.c
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_LED_STRIP=0 -DHOST_PWM=0 -DHOST_LED=0 -o $@ $<
clean:
	$(RM) $(BINS)
