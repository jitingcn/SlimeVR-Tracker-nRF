CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS += -I. -I../../../src

.PHONY: check clean

check: test_sessions
	./test_sessions

test_sessions: test_sessions.c ../../../src/sensor/diagnostics.c \
		../../../src/sensor/diagnostics.h zephyr/kernel.h \
		zephyr/logging/log.h zephyr/sys/printk.h Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< -lm

clean:
	$(RM) test_sessions
