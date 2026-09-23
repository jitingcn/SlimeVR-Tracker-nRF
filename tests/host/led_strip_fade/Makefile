CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror -I.

.PHONY: check clean

check: test_led_strip_fade
	./test_led_strip_fade

test_led_strip_fade: test_led_strip_fade.c ../../../src/system/led_strip_fade.h Makefile
	$(CC) $(CFLAGS) -o $@ $<

clean:
	$(RM) test_led_strip_fade
