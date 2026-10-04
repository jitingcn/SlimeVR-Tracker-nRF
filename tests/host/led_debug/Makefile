CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
FLAGS = $(CFLAGS) -DCONFIG_LED_DEBUG=1 -I. -I../led_sync -I../../../src
SOURCES = test_debug.c ../../../src/system/led_debug.c ../../../src/system/led_policy.c \
          ../../../src/system/led_behavior.c ../../../src/system/led_timeline.c ../../../src/parse_args.c
DEPS = Makefile $(SOURCES) ../led_sync/test_runtime.h ../led_sync/globals.h \
       ../../../src/system/led.c ../../../src/system/led.h ../../../src/system/led_internal.h \
       ../../../src/system/led_debug.h ../../../src/system/led_policy.h \
       ../../../src/system/led_sync.h ../../../src/parse_args.h zephyr/sys/printk.h \
       ../led_sync/connection/esb.h $(wildcard ../led_sync/zephyr/*.h ../led_sync/zephyr/*/*.h)
BINS = test_debug_nrf52 test_debug_nrf54 test_debug_network

.PHONY: check clean
check: $(BINS)
	./test_debug_nrf52
	./test_debug_nrf54
	./test_debug_network

test_debug_nrf52: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_SYS_CLOCK_TICKS_PER_SEC=32768 $(SOURCES) -o $@

test_debug_nrf54: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_SYS_CLOCK_TICKS_PER_SEC=31250 $(SOURCES) -o $@

test_debug_network: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 $(SOURCES) -o $@

clean:
	$(RM) $(BINS)
