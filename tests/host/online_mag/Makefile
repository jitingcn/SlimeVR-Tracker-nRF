CC ?= cc
SOURCE_ROOT ?= ../../..
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CPPFLAGS += -I. -I$(SOURCE_ROOT)/src
SANITIZERS ?= -fsanitize=address,undefined -fno-sanitize-recover=undefined
LDLIBS += -lm
REAL_SOURCES := $(SOURCE_ROOT)/src/sensor/calibration/mag_fit.c $(SOURCE_ROOT)/src/sensor/magneto/magneto1_4.c $(SOURCE_ROOT)/src/sensor/magneto/mymathlib_matrix.c
PRODUCTION_INCLUDE := .generated/production.inc
PRODUCTION_SOURCES := $(SOURCE_ROOT)/src/util.c $(SOURCE_ROOT)/src/sensor/calibration/magneto.c $(SOURCE_ROOT)/src/sensor/calibration/calibration.c

.PHONY: check clean test_online_mag libonline_mag.so
check: test_online_mag libonline_mag.so
	./test_online_mag
	python3 test_identity_restore.py
	python3 test_recovery.py
	python3 test_clear.py
	python3 test_usability.py

$(PRODUCTION_INCLUDE): extract_production.py $(PRODUCTION_SOURCES)
	python3 extract_production.py $@ $(SOURCE_ROOT)

test_online_mag: test_online_mag.c $(PRODUCTION_INCLUDE)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANITIZERS) $< -o $@ $(LDLIBS)

# ctypes/external-trajectory entry: actual lifecycle + actual robust fitter and
# Magneto initializer. No fixed-candidate mock. Input matrices are float[12],
# vectors float[3]; fixture_feed takes uint32 dt milliseconds. No sanitizer here
# because Python does not preload ASan; standalone check above is sanitized.
libonline_mag.so: test_online_mag.c $(REAL_SOURCES) $(PRODUCTION_INCLUDE)
	$(CC) $(CPPFLAGS) $(CFLAGS) -DREAL_FIT -DFIXTURE_LIBRARY -fPIC -shared test_online_mag.c $(REAL_SOURCES) -o $@ $(LDLIBS)

clean:
	$(RM) test_online_mag libonline_mag.so
	$(RM) -r .generated
