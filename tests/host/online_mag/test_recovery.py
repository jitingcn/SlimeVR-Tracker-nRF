"""Real lifecycle + real fitter: bootstrap, recovery, scale-free no-op/environment.
No magnetic-fusion cleanliness feedback is provided to the lifecycle.
"""
import ctypes as C
import math
import random
from pathlib import Path

lib = C.CDLL(str(Path(__file__).with_name("libonline_mag.so")))
Vec = C.c_float * 3
Matrix = C.c_float * 12
lib.fixture_reset.argtypes = [C.c_uint32, C.c_int]
lib.fixture_feed.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_float), C.c_int, C.c_uint32]
for name in ("fixture_model", "fixture_live", "fixture_retained"):
    getattr(lib, name).argtypes = [C.POINTER(C.c_float)]
lib.fixture_reference.argtypes = [C.c_float, C.c_float]
lib.fixture_reference_norm.restype = C.c_float
lib.fixture_reference_dip.restype = C.c_float
IDENTITY = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]
BASE = [.08, -.04, .03]


def matrix(bias, scale=1):
    return list(bias) + [scale, 0, 0, 0, scale, 0, 0, 0, scale]


def read_matrix(name):
    out = Matrix()
    getattr(lib, name)(out)
    return list(out)


def run(bias, *, initial=None, strength=.5, gravity=True, dip=.6,
        noise=0, stale=False, nonuniform=False, transient=False, revert=False):
    lib.fixture_reset(1000, 0)
    if initial is not None:
        lib.fixture_model(Matrix(*initial))
    before = read_matrix("fixture_retained")
    rng = random.Random(8743)
    outcomes = set()
    spike = False
    reverted = False
    for i in range(2400):
        unit = [rng.gauss(0, 1) for _ in range(3)]
        length = math.sqrt(sum(x*x for x in unit))
        unit = [x/length for x in unit]
        perpendicular = [-unit[1], unit[0], 0]
        length = math.hypot(*perpendicular[:2])
        local_dip = (.25 if unit[0] > 0 else .8) if nonuniform else dip
        up = [local_dip*x + math.sqrt(1-local_dip**2)*y/length
              for x, y in zip(unit, perpendicular)]
        radius = strength * (1 + (noise if i % 2 else -noise))
        if revert and lib.fixture_phase() == 5:
            bias = BASE
            reverted = True
        raw = [radius*x + b for x, b in zip(unit, bias)]
        if transient and lib.fixture_phase() == 5 and not spike:
            raw[0] += 2
            spike = True
        lib.fixture_feed(Vec(*raw), Vec(*up), gravity, 40)
        if i == 0 and initial is not None:
            lib.fixture_reference(.5, -math.asin(.6))
        # Keep fusion's old reference stale/rejected, even after notifications.
        if stale:
            lib.fixture_reference(.19, 1.1)
        if i % 25 == 0:
            lib.fixture_check()
        outcomes.add(lib.fixture_outcome())
        if transient and spike:
            assert lib.fixture_dirty() == 0
            assert read_matrix("fixture_live") == before
            return before, outcomes
        if lib.fixture_dirty():
            return before, outcomes
        if lib.fixture_outcome() in (1, 2):
            assert not revert or reverted, "transient replay never reached replacement probation"
            return before, outcomes
    assert not revert or reverted, "transient replay never reached replacement probation"
    return before, outcomes


for gravity in (True, False):
    run(BASE, gravity=gravity)
    assert lib.fixture_dirty() == 1, "no-reference bootstrap failed"
    fitted = read_matrix("fixture_retained")
    assert max(abs(a-b) for a, b in zip(fitted, matrix(BASE))) < .004
    if not gravity:
        assert not lib.fixture_dip_known()
        assert lib.fixture_reference_norm() == 0

shift = [-.11177, .18174, -.01570]
changed = [a+b for a, b in zip(BASE, shift)]
run(changed, initial=matrix(BASE), stale=True)
assert lib.fixture_dirty() == 1, "stale old reference blocked large-bias recovery"
assert max(abs(a-b) for a, b in zip(read_matrix("fixture_retained"), matrix(changed))) < .004

# Legacy coefficient scale and field strength are not calibration errors.
legacy = matrix(BASE, 1.1)
before, outcomes = run(BASE, initial=legacy, strength=.65, dip=-.2)
assert lib.fixture_dirty() == 0 and lib.fixture_updates() == 0
assert read_matrix("fixture_live") == before == read_matrix("fixture_retained")
assert 2 in outcomes, "stable changed norm/dip did not publish environment-only outcome"
assert lib.fixture_dip_known()
assert abs(lib.fixture_reference_norm() - .715) < .003
assert abs(lib.fixture_reference_dip() - math.asin(.2)) < .01

before, outcomes = run(BASE, initial=matrix(BASE), noise=.008)
assert lib.fixture_dirty() == 0 and lib.fixture_updates() == 0
assert read_matrix("fixture_retained") == before == read_matrix("fixture_live")
assert outcomes & {1, 2}, "noise-floor model needs explicit unchanged outcome"

run(BASE, nonuniform=True)
assert lib.fixture_dirty() == 0, "orientation-dependent field passed gravity veto"
run(BASE, transient=True)
assert lib.fixture_dirty() == 0, "transient probation spike was persisted"

# A temporary bias can survive training+holdout but disappear during probation.
# The restored healthy previous model must not exempt the candidate's error.
run([BASE[0] + .04, BASE[1], BASE[2]], initial=matrix(BASE), gravity=False, revert=True)
assert lib.fixture_dirty() == 0, "healthy previous model bypassed candidate probation"
run([BASE[0] + .02, BASE[1], BASE[2]], initial=matrix(BASE), gravity=False, revert=True)
assert lib.fixture_dirty() == 0, "small transient passed absolute quality after its benefit disappeared"
print("online_mag real solver: no-ref/no-gravity bootstrap, large-bias stale-ref recovery, environment/noise no-op, nonuniform/spike rejection")