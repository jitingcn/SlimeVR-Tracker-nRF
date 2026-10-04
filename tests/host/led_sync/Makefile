CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CORE = ../../../src/system/led_policy.c ../../../src/system/led_behavior.c ../../../src/system/led_timeline.c
DEPS = Makefile test_led_sync.c test_runtime.h globals.h ../../../src/system/led.c ../../../src/system/led.h \
	../../../src/system/led_internal.h ../../../src/system/led_policy.h ../../../src/system/led_sync.h $(CORE) \
	connection/esb.h $(wildcard zephyr/*.h zephyr/*/*.h)
BINS = test_semantic_local test_semantic_sync test_semantic_nrf54 test_semantic_debug
.PHONY: check clean
check: $(BINS)
	@set -e; for test in $(BINS); do ./$$test; done
test_semantic_local: $(DEPS)
	$(CC) $(CFLAGS) -I. -o $@ test_led_sync.c $(CORE)
test_semantic_sync: $(DEPS)
	$(CC) $(CFLAGS) -I. -DCONFIG_LED_NETWORK_SYNC=1 -o $@ test_led_sync.c $(CORE)
test_semantic_nrf54: $(DEPS)
	$(CC) $(CFLAGS) -I. -DCONFIG_LED_NETWORK_SYNC=1 -DCONFIG_SYS_CLOCK_TICKS_PER_SEC=31250 -o $@ test_led_sync.c $(CORE)
test_semantic_debug: $(DEPS)
	$(CC) $(CFLAGS) -I. -DCONFIG_LED_DEBUG=1 -o $@ test_led_sync.c $(CORE)
clean:
	$(RM) $(BINS)
