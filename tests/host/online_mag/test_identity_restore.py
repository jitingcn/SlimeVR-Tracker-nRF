"""A confirmed identity must not become a new bootstrap after warm/read restore.

Actual online_mag + mag_fit + Magneto shared library, not the candidate mock.
An already-correct sphere must finish unchanged under scale-free geometry
assessment, not be treated as an uncalibrated identity requiring a new fit.
"""
import ctypes as C
import math
from pathlib import Path
import random

lib = C.CDLL(str(Path(__file__).with_name("libonline_mag.so")))
Vec = C.c_float * 3
Matrix = C.c_float * 12
lib.fixture_reset.argtypes = [C.c_uint32, C.c_int]
lib.fixture_restore_identity.argtypes = [C.c_int]
lib.fixture_feed.argtypes = [C.POINTER(C.c_float), C.POINTER(C.c_float), C.c_int, C.c_uint32]
lib.fixture_live.argtypes = [C.POINTER(C.c_float)]
lib.fixture_retained.argtypes = [C.POINTER(C.c_float)]
lib.fixture_reference.argtypes = [C.c_float, C.c_float]
identity = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]

for started in (0, 1):
    lib.fixture_reset(1000, 0)
    lib.fixture_restore_identity(started)
    rng = random.Random(742 + started)
    for i in range(2500):
        u = [rng.gauss(0, 1) for _ in range(3)]
        length = math.sqrt(sum(x*x for x in u))
        u = [x/length for x in u]
        perp = [-u[1], u[0], 0]
        length = math.hypot(perp[0], perp[1])
        up = [.6*x + .8*y/length for x, y in zip(u, perp)]
        lib.fixture_feed(Vec(*(.5*x for x in u)), Vec(*up), 1, 40)
        if i == 0:
            lib.fixture_reference(.5, -math.asin(.6))
        if i % 50 == 0:
            lib.fixture_check()
        # No trial or candidate can overwrite persisted confirmed coefficients.
        retained = Matrix()
        lib.fixture_retained(retained)
        assert list(retained) == identity
    assert lib.fixture_updates() == 1, "confirmed identity incorrectly treated as bootstrap"
    assert lib.fixture_dirty() == 0, "unchanged sphere cannot justify an online update"
    live = Matrix()
    lib.fixture_live(live)
    assert list(live) == identity
print("online_mag actual solver: confirmed identity startup/queued restore preserved")
