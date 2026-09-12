CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror -pthread -DHOST_CONCURRENCY_STRESS -I.

.PHONY: check clean

check: test_sensor_snapshot
	./test_sensor_snapshot

test_sensor_snapshot: test_sensor_snapshot.c ../../../src/connection/sensor_data_snapshot.h \
		zephyr/kernel.h zephyr/sys/atomic.h Makefile
	$(CC) $(CFLAGS) -o $@ $<

clean:
	$(RM) test_sensor_snapshot
