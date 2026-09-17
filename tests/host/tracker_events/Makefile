.PHONY: check clean

check:
	python3 check_protocol_sync.py
	CC="$(CC)" python3 run.py
	CC="$(CC)" python3 test_tcal.py
	CC="$(CC)" python3 test_requests.py
	CC="$(CC)" python3 test_mag.py

# All generated objects live in temporary directories.
clean:
