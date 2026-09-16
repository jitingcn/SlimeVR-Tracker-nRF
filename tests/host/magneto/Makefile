CC ?= cc
SOURCE_ROOT ?= ../../..
LEGACY ?= 0
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -fsanitize=address,undefined -fno-sanitize-recover=undefined -I. -I$(SOURCE_ROOT)/src/sensor/magneto -DTEST_LEGACY=$(LEGACY)
LDLIBS += -lm -fsanitize=address,undefined
LDFLAGS += -Wl,--wrap=QR_Hessenberg_Matrix -Wl,--wrap=Hessenberg_Form_Elementary
LDFLAGS += -Wl,--wrap=Choleski_LU_Decomposition -Wl,--wrap=Choleski_LU_Inverse

SOURCES := test_magneto.c $(SOURCE_ROOT)/src/sensor/magneto/magneto1_4.c $(SOURCE_ROOT)/src/sensor/magneto/mymathlib_matrix.c
HEADERS := zephyr/kernel.h $(SOURCE_ROOT)/src/sensor/magneto/magneto1_4.h $(SOURCE_ROOT)/src/sensor/magneto/mymathlib_matrix.h

# Always rebuild this small probe: SOURCE_ROOT and LEGACY may change without
# source timestamps changing when comparing a preserved snapshot.
.PHONY: check clean test_magneto
check: test_magneto
	./test_magneto

test_magneto: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -c -o test_magneto.o test_magneto.c
	$(CC) $(CFLAGS) -c -o magneto1_4.o $(SOURCE_ROOT)/src/sensor/magneto/magneto1_4.c
	$(CC) $(CFLAGS) -c -o mymathlib_matrix.o $(SOURCE_ROOT)/src/sensor/magneto/mymathlib_matrix.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_magneto.o magneto1_4.o mymathlib_matrix.o $(LDLIBS)

clean:
	$(RM) test_magneto test_magneto.o magneto1_4.o mymathlib_matrix.o
