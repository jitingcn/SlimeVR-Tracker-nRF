"""Real mag-clear + structural validator + fitter: erased is not calibrated."""
import ctypes as C
import math
from pathlib import Path

lib = C.CDLL(str(Path(__file__).with_name("libonline_mag.so")))
Vec = C.c_float * 3
Matrix = C.c_float * 12


class Diagnostics(C.Structure):
    _fields_ = [
        ("old_rms", C.c_float), ("new_rms", C.c_float),
        ("worst_cell_rms", C.c_float), ("max_radial_error", C.c_float),
        ("old_dip_sd", C.c_float), ("new_dip_sd", C.c_float), ("dip_delta", C.c_float),
        ("phase_age_ms", C.c_uint32),
        ("fit_errno", C.c_int), ("radial_count", C.c_uint16), ("dip_count", C.c_uint16),
        *((name, C.c_uint8) for name in ("phase", "outcome", "rejection", "radial_cells",
                                       "dip_cells", "radial_poles", "dip_poles", "score_phase")),
        ("last_gate", C.c_uint8),
        ("has_model", C.c_bool), ("trial", C.c_bool), ("score_valid", C.c_bool),
    ]


lib.fixture_reset.argtypes = [C.c_uint32, C.c_int]
lib.fixture_feed.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_float), C.c_int, C.c_uint32]
for name in ("fixture_model", "fixture_live", "fixture_retained"):
    getattr(lib, name).argtypes = [C.POINTER(C.c_float)]
lib.sensor_calibration_online_mag_diagnostics.argtypes = [C.POINTER(Diagnostics)]
lib.mag_bainv_structurally_ok.argtypes = [C.POINTER(C.c_float), C.c_float]
lib.mag_bainv_structurally_ok.restype = C.c_bool
IDENTITY = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]
BIAS = [.12672, .20461, -.21345]
GAIN = [1.1, .95, 1.05]


def diagnostic():
    result = Diagnostics()
    lib.sensor_calibration_online_mag_diagnostics(C.byref(result))
    return result


def matrix(name):
    result = Matrix()
    getattr(lib, name)(result)
    return list(result)


def sample(i):
    # Continuous physical body rotation; magnetic field and gravity share R^T.
    t = i * .04
    roll, pitch, yaw = .83*t, 1.48*math.sin(.31*t), .53*t
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    up = [-sp, cp*sr, cp*cr]
    north = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr]
    dip = math.radians(48)
    unit = [math.cos(dip)*n + math.sin(dip)*g for n, g in zip(north, up)]
    return [b + .5*u/a for b, u, a in zip(BIAS, unit, GAIN)], up


assert lib.mag_bainv_structurally_ok(Matrix(*IDENTITY), 0)
valid = BIAS + [1.1, .01, .005, .01, 1.15, -.005, .005, -.005, 1.16]
assert lib.mag_bainv_structurally_ok(Matrix(*valid), 0)
for invalid in (
    [0]*12,
    [0]*3 + [-1, 0, 0, 0, -1, 0, 0, 0, -1],
    [0]*3 + [1]*9,  # positive diagonal is not sufficient: rank one
    [0]*3 + [1, 1.1, 0, 1.1, 1, 0, 0, 0, 1],  # indefinite
    [0]*3 + [1, .2, 0, 0, 1, 0, 0, 0, 1],  # asymmetric
    [math.nan] + IDENTITY[1:],
    IDENTITY[:3] + [math.inf] + IDENTITY[4:],
):
    assert not lib.mag_bainv_structurally_ok(Matrix(*invalid), 0), invalid

for running in (False, True):
    lib.fixture_reset(1000, int(running))
    if running:
        raw, up = sample(0)
        lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)
    lib.fixture_clear()  # Actual production sensor_calibration_clear_mag(NULL,true).
    raw, up = sample(0)
    lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)  # Sensor acknowledges queued clear.
    assert matrix("fixture_live") == [0]*12 == matrix("fixture_retained")
    assert not diagnostic().has_model and not diagnostic().score_valid
    saw_trial = False
    for i in range(1, 8000):
        raw, up = sample(i)
        lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)
        if i % 25 == 0:
            lib.fixture_check()
        d = diagnostic()
        if d.trial:
            saw_trial = True
            assert d.has_model and matrix("fixture_retained") == [0]*12
        if lib.fixture_dirty():
            break
    assert lib.fixture_dirty() == 1 and saw_trial, "cleared online bootstrap never confirmed"
    fitted = matrix("fixture_retained")
    expected = BIAS + [GAIN[0], 0, 0, 0, GAIN[1], 0, 0, 0, GAIN[2]]
    assert max(abs(a-b) for a, b in zip(fitted, expected)) < .003
    # Clearing after an actual scored, confirmed episode discards its diagnostics.
    assert diagnostic().score_valid
    lib.fixture_clear()
    lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)
    d = diagnostic()
    assert not d.has_model and not d.trial and not d.score_valid
    assert d.radial_count == d.dip_count == 0 and lib.fixture_updates() == 0


def enter_scored_holdout():
    lib.fixture_reset(1000, 0)
    for i in range(8000):
        raw, up = sample(i)
        lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)
        if i % 25 == 0:
            lib.fixture_check()
        if lib.fixture_phase() == 4:
            break
    assert lib.fixture_phase() == 4
    # Hold one valid direction long enough to score without earning coverage.
    for _ in range(210):
        lib.fixture_feed(Vec(*raw), Vec(*up), 1, 40)
    d = diagnostic()
    assert d.score_valid and d.radial_count >= 200 and d.score_phase == 4
    assert d.radial_poles != 0x3f
    return raw, up


for nonfinite in (False, True):
    raw, up = enter_scored_holdout()
    bad = list(raw)
    bad[0] = math.nan if nonfinite else bad[0] + 2
    lib.fixture_feed(Vec(*bad), Vec(*up), 1, 40)
    d = diagnostic()
    assert d.rejection == (8 if nonfinite else 2)  # SAMPLE versus RADIAL
    assert not d.score_valid and d.radial_count == d.dip_count == 0
    assert (d.max_radial_error == 0) if nonfinite else (d.max_radial_error > .3)
    assert lib.fixture_dirty() == 0


print("online_mag actual clear: zero/singular rejection, smooth cold/running bootstrap, trial provenance and stale-score reset passed")
