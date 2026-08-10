CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror -pthread -DHOST_CONCURRENCY_STRESS -I.

.PHONY: check clean

check: test_sensor_snapshot test_persistence_interleavings
	./test_sensor_snapshot
	./test_persistence_interleavings

test_sensor_snapshot: test_sensor_snapshot.c
	$(CC) $(CFLAGS) -o $@ $<

test_persistence_interleavings: test_persistence_interleavings.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	$(RM) test_sensor_snapshot test_persistence_interleavings
