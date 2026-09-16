"""Actual-source lifecycle/solver regressions; no injected candidates or gate mocks.
Run one named case to replay against an isolated pre-change source snapshot.
"""
import ctypes as C
import math
import sys
from pathlib import Path

lib = C.CDLL(str(Path(__file__).with_name("libonline_mag.so")))
Vec = C.c_float * 3
Matrix = C.c_float * 12
lib.fixture_reset.argtypes = [C.c_uint32, C.c_int]
lib.fixture_feed.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_float), C.c_int, C.c_uint32]
lib.fixture_pool_newer_than.argtypes = [C.c_uint32]
lib.fixture_now.restype = C.c_uint32
lib.fixture_reference.argtypes = [C.c_float, C.c_float]
for name in ("fixture_model", "fixture_live", "fixture_retained"):
    getattr(lib, name).argtypes = [C.POINTER(C.c_float)]
for name in ("fixture_old_rms", "fixture_new_rms", "fixture_reference_norm"):
    getattr(lib, name).restype = C.c_float
BASE = [.08, -.04, .03]
CHANGED = [-.03177, .14174, .01430]
VALIDATING, PROBATION, CONFIRM = 4, 5, 6


def model(bias):
    return list(bias) + [1, 0, 0, 0, 1, 0, 0, 0, 1]


def matrix(name="fixture_retained"):
    value = Matrix()
    getattr(lib, name)(value)
    return list(value)


def reset(initial=None):
    lib.fixture_reset(1000, 0)
    if initial is not None:
        lib.fixture_model(Matrix(*initial))


def direction(i, upper=False):
    # All 24 directional cells, equal occupancy, exact deterministic pole set.
    axis, signs = i % 3, (i // 3) % 8
    unit = [(1 if j == axis else .3) * (-1 if signs & (1 << j) else 1)
            for j in range(3)]
    if upper:
        unit[2] = abs(unit[2])
    norm = math.sqrt(sum(x*x for x in unit))
    return [x/norm for x in unit]


def feed(i, bias=BASE, *, noise=0, dip=.6, upper=False, gravity=True, smooth=False):
    if smooth:
        t = i * .04
        r, p, y = .83*t, 1.48*math.sin(.31*t), .53*t
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        up = [-sp, cp*sr, cp*cr]
        north = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr]
        unit = [math.sqrt(1-dip*dip)*n + dip*g for n, g in zip(north, up)]
    else:
        unit = direction(i, upper)
        cross = [-unit[1], unit[0], 0]
        norm = math.hypot(*cross[:2])
        up = [dip*u + math.sqrt(1-dip*dip)*v/norm for u, v in zip(unit, cross)]
    # Alternate noise on complete directional sweeps, not sign/cell identity.
    radius = .5 * (1 + noise * (1 if (i // 24) % 2 else -1))
    lib.fixture_feed(Vec(*(radius*u+b for u, b in zip(unit, bias))), Vec(*up), gravity, 40)


def train(initial=None):
    reset(initial)
    for i in range(2400):
        feed(i)
        if i % 25 == 0:
            lib.fixture_check()
        if lib.fixture_phase() == VALIDATING:
            return i + 1
    raise AssertionError("actual solver never entered validation")


def until_phase(i, phase, **kwargs):
    for j in range(i, i + 1000):
        feed(j, **kwargs)
        if lib.fixture_phase() == phase:
            return j + 1
    raise AssertionError(f"never reached phase {phase}; outcome={lib.fixture_outcome()}")


def continuous_pool():
    i = train()
    retained = matrix()
    start = lib.fixture_now()
    i = until_phase(i, PROBATION)
    assert lib.fixture_pool_newer_than(start) >= 32, "validation stopped raw collection"
    assert matrix() == retained and lib.fixture_dirty() == 0
    trial = matrix("fixture_live")
    start = lib.fixture_now()
    until_phase(i, CONFIRM)
    assert lib.fixture_pool_newer_than(start) >= 32, "probation stopped raw collection"
    assert matrix("fixture_live") == trial and matrix() == retained
    assert lib.fixture_check() and lib.fixture_dirty() == 1


def continuous_shift():
    reset(model(BASE))
    before = matrix()
    # Shift during the first holdout: the old pool is already populated and a
    # real fit has started. Never reset, pause, or clear at the bias transition.
    shifted = False
    first_shift = None
    saw_mixed_failure = False
    for i in range(12000):
        if not shifted and lib.fixture_phase() == VALIDATING:
            shifted, first_shift = True, i
        feed(i, CHANGED if shifted else BASE, smooth=True)
        if shifted:
            lib.fixture_reference(.19, 1.1)  # deliberately stale fusion reference
        if i % 25 == 0:
            lib.fixture_check()
        if shifted and lib.fixture_outcome() == 4:
            saw_mixed_failure = True
        if lib.fixture_dirty():
            break
    assert shifted and saw_mixed_failure, "did not exercise mixed-history retry"
    assert lib.fixture_dirty() == 1, f"continuous shift never recovered after sample {first_shift}"
    fitted = matrix()
    assert fitted != before and fitted == matrix("fixture_live")
    # Mixed training history can produce an accurate but nonexact ellipsoid.
    # Test what fusion consumes on an independent, uniformly distributed sphere,
    # not exact coefficients or the training/validation trajectory's own score.
    errors = []
    for n in range(4096):
        z = 1 - 2*(n + .5)/4096
        angle = n * math.pi * (3 - math.sqrt(5))
        unit = [math.sqrt(1-z*z)*math.cos(angle), math.sqrt(1-z*z)*math.sin(angle), z]
        centered = [.5*unit[j] + CHANGED[j] - fitted[j] for j in range(3)]
        corrected = [sum(fitted[3 + 3*j + k]*centered[j] for j in range(3))
                     for k in range(3)]
        errors.append(math.sqrt(sum(x*x for x in corrected))/.5 - 1)
    rms = math.sqrt(sum(x*x for x in errors)/len(errors))
    maximum = max(map(abs, errors))
    assert rms <= .012 and maximum <= .03, (rms, maximum)


def no_benefit():
    i = train(model([BASE[0] + .005, BASE[1], BASE[2]]))
    before = matrix()
    for j in range(i, i + 500):
        feed(j, noise=.0198, gravity=False)
        assert lib.fixture_phase() != PROBATION, "noise-floor improvement entered replacement probation"
        if lib.fixture_outcome() == 1:
            break
    assert lib.fixture_outcome() == 1, "no-benefit decision stayed pending"
    assert .019 < lib.fixture_new_rms() < .021
    assert .0195 < lib.fixture_old_rms() < .022
    assert matrix() == before == matrix("fixture_live")
    assert lib.fixture_dirty() == lib.fixture_updates() == 0


def bad_quality():
    i = train()
    start = lib.fixture_now()
    for j in range(i, i + 1000):
        feed(j, noise=.08, gravity=False)
        if lib.fixture_outcome() == 4:
            break
    assert lib.fixture_outcome() == 4, "all-covered bad radial quality remained pending"
    assert lib.fixture_now() - start < 45000
    assert lib.fixture_rejection() == 2 and lib.fixture_poles() == 0x3f
    assert lib.fixture_dirty() == 0


def five_poles():
    i = train()
    i = until_phase(i, PROBATION)
    until_phase(i, CONFIRM, upper=True, gravity=False)
    assert lib.fixture_poles().bit_count() == 5
    assert lib.fixture_check() and lib.fixture_dirty() == 1
    # Identical hemisphere is insufficient as the first independent holdout.
    i = train()
    for j in range(i, i + 900):
        feed(j, upper=True, gravity=False)
    assert lib.fixture_phase() == VALIDATING and lib.fixture_dirty() == 0


def dip_transition():
    i = train()
    i = until_phase(i, PROBATION)
    until_phase(i, CONFIRM, dip=-.2)
    assert lib.fixture_check() and lib.fixture_dirty() == 1
    feed(i + 1000, dip=-.2)  # Sensor consumes confirmation's unknown-reference notice.
    assert not lib.fixture_dip_known(), "different window means fabricated a stable magnetic reference"
    assert lib.fixture_reference_norm() == 0


def unstable_environment():
    i = train(model(BASE))
    before = matrix()
    # Changed but internally coherent first-window dip warrants environment-only
    # observation; a different second window must finish, never save coefficients.
    i = until_phase(i, PROBATION, dip=-.2)
    start = lib.fixture_now()
    for j in range(i, i + 1000):
        feed(j, dip=.3)
        if lib.fixture_outcome() == 1:
            break
    assert lib.fixture_outcome() == 1 and lib.fixture_now() - start < 45000
    assert matrix() == before == matrix("fixture_live") and lib.fixture_dirty() == 0


CASES = {fn.__name__: fn for fn in (continuous_pool, continuous_shift, no_benefit,
          bad_quality, five_poles, dip_transition, unstable_environment)}
for name in sys.argv[1:] or CASES:
    CASES[name]()
    print(f"online_mag usability: {name} passed")
