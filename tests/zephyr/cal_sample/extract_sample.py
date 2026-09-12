#!/usr/bin/env python3
"""Extract all production sample state/functions; kernel calls remain real."""
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text()
body = source[source.index("#define CAL_SAMPLE_QUEUE_CAPACITY"):]
# Permit running the same behavioral assertions against a saved pre-fix source.
# This adapter only names its existing reset operation; it adds no admission.
if "void sensor_calibration_samples_reset(void)" in body:
    body += """
void sensor_calibration_samples_begin(uint8_t channels)
{
    ARG_UNUSED(channels);
    sensor_calibration_samples_reset();
}
void sensor_calibration_samples_end(void)
{
    sensor_calibration_samples_reset();
}
"""
Path(sys.argv[2]).write_text(body)
