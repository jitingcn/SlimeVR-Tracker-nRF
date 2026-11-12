CC ?= cc
CFLAGS := -std=c11 -Wall -Wextra -Werror -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -I../../../src
TMP := $(shell mktemp -d 2>/dev/null || echo /tmp/tracker-relay-host)

.PHONY: check clean
check:
	$(CC) $(CFLAGS) test_b2_deadline.c -o $(TMP)/b2_deadline
	$(TMP)/b2_deadline
	python3 run_peak.py
	python3 run_layout_failure.py
	CC="$(CC)" python3 run_quiesce.py
	$(CC) $(CFLAGS) test_link.c -o $(TMP)/test_link
	$(TMP)/test_link
	CC="$(CC)" python3 run_tdma.py
	CC="$(CC)" python3 run_schedule_equivalence.py
	CC="$(CC)" python3 run_runtime.py
	CC="$(CC)" python3 run_presence.py
	CC="$(CC)" python3 run_control_window.py
	CC="$(CC)" python3 run_own_schedule.py
	CC="$(CC)" python3 run_radio.py
	CC="$(CC)" python3 run_capture_time.py
	CC="$(CC)" python3 run_submission.py
	CC="$(CC)" python3 run_source_guard.py
	CC="$(CC)" python3 run_producer.py
	python3 budget.py
	python3 check_protocol_sync.py

clean:
	@:
