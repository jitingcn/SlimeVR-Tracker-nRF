CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
CPPFLAGS += -I. -I../../../src
LDLIBS += -pthread

.PHONY: check clean

check: test_raw_retx test_channel_control
	./test_raw_retx
	./test_channel_control

test_raw_retx: test_raw_retx.c ../../../src/connection/raw_retx.h Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDLIBS)

test_channel_control: test_channel_control.c ../../../src/connection/channel_control.c \
		../../../src/connection/channel_control.h ../../../src/connection/esb.h \
		../../../src/retained.h ../../../src/system/system.h \
		zephyr/kernel.h esb.h Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	$(RM) test_raw_retx test_channel_control
