CC ?= cc
SOURCE_ROOT ?= ../../..
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -fsanitize=address,undefined -fno-sanitize-recover=undefined -I../magneto -I$(SOURCE_ROOT)/src
LDLIBS += -lm -fsanitize=address,undefined

.PHONY: check clean test_mag_fit
check: test_mag_fit
	./test_mag_fit

test_mag_fit: test_mag_fit.c $(SOURCE_ROOT)/src/sensor/calibration/mag_fit.c $(SOURCE_ROOT)/src/sensor/calibration/mag_fit.h
	$(CC) $(CFLAGS) -c -o test_mag_fit.o test_mag_fit.c
	$(CC) $(CFLAGS) -c -o mag_fit.o $(SOURCE_ROOT)/src/sensor/calibration/mag_fit.c
	$(CC) $(CFLAGS) -c -o magneto1_4.o $(SOURCE_ROOT)/src/sensor/magneto/magneto1_4.c
	$(CC) $(CFLAGS) -c -o mymathlib_matrix.o $(SOURCE_ROOT)/src/sensor/magneto/mymathlib_matrix.c
	$(CC) $(CFLAGS) -o $@ test_mag_fit.o mag_fit.o magneto1_4.o mymathlib_matrix.o $(LDLIBS)

clean:
	$(RM) test_mag_fit test_mag_fit.o mag_fit.o magneto1_4.o mymathlib_matrix.o
