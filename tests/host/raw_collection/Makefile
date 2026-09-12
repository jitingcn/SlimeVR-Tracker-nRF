CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -fsanitize=undefined -fno-sanitize-recover=undefined -I. -I../../../src
LDLIBS += -lm -fsanitize=undefined

SOURCES := test_raw_collection.c ../../../src/sensor/raw_collection.c
HEADERS := ../../../src/sensor/raw_collection.h ../../../src/connection/connection.h ../../../src/util.h zephyr/sys/atomic.h

.PHONY: check clean

check: test_raw_collection
	./test_raw_collection

test_raw_collection: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $(SOURCES) $(LDLIBS)

clean:
	$(RM) test_raw_collection
