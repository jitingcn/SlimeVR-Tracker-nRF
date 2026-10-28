.PHONY: check cycle-rebase clean

check:
	rtk proxy python3 run.py
	rtk proxy python3 test_cycle_rebase.py
	rtk uv run --project ../../../../vqf_experiments test_output_stop.py

cycle-rebase:
	rtk proxy python3 test_cycle_rebase.py

# Native smoke builds use TemporaryDirectory; exports are opt-in.
clean:
	@:
