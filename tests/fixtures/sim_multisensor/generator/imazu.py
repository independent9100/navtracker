"""The Imazu 22 — the canonical ship-encounter benchmark as fixed-geometry
sim scenarios (``imazu_01`` .. ``imazu_22``).

WHAT THESE ARE
--------------
The "Imazu problem" (Imazu, H., 1987, PhD thesis, Univ. of Tokyo) is the
standard set of 22 encounter situations used throughout the COLREG /
collision-avoidance literature: own-ship plus 1-3 target ships in every
combination of head-on / crossing / overtaking. Cases 1-4 are single-target
(the four pure COLREG encounters), 5-11 are two-target (7 cases), 12-22 are
three-target (11 cases): 4 + 7 + 11 = 22.

We are a TRACKER suite, not a COLAV planner: own-ship steams a fixed course and
never manoeuvres, and every target holds a constant course/speed. The encounters
are allowed to pass close or even through collision geometry; that closeness
through CPA is exactly the identity-stability stress this suite measures. Radar +
AIS arm on every vessel (no camera). Only the Layer-2 observation models
(``sensors.py``) are shared with the sim_ms battery; the geometry here is
explicit (``ExplicitInitial`` in ``truth.py``), not trafficgen-sampled.

GEOMETRY SOURCE (primary, verbatim)
-----------------------------------
CORALL repository ``src/utils/imazu_cases.py`` (Klins101/CORALL, MIT license),
https://github.com/Klins101/CORALL — the runnable encoding from the
Sarhadi/Naeem group. Verified by direct download (2026-07-08). The raw
per-target arithmetic below is transcribed byte-for-byte from that file so it can
be diffed against the source; only ``Case 23`` (a non-canonical add-on, not part
of the classic 22) is dropped.

CORALL frame: own-ship at (0,0) heading 0deg = due EAST (+x); each target is
``[[x_nm, y_nm], heading_deg]`` with heading measured CCW from +x (0=E, 90=N,
180=W, 270=S) and +y = North = own-ship's port side. We keep that frame directly:
own-ship's course is set to marine 090 (East), so CORALL (x, y) maps 1:1 onto
ENU (East=x, North=y) and a CORALL heading ``h`` becomes marine course
``(90 - h) mod 360``. See ``_to_initial``.

SPEED CONVENTION (and why the source's numbers can't be used verbatim)
---------------------------------------------------------------------
CORALL's ``get_obstacle_data`` hardcodes non-physical scalars (own 43.3, every
target 18.52) — as m/s those are ~84 / 36 kn, absurd; they are model-scaled
units, not speeds. But their RATIO is meaningful: CORALL's crossing geometry is
tuned so a target collides with own-ship iff ``v_target / v_own = 18.52/43.3 =
0.4277``. (Check: Case 2 target at (5, -2.14) heading North reaches own-ship's
x-axis track exactly when ``2.14/v_t = 5/v_own`` => ratio 0.428.) That single
ratio reproduces every designed CPA — crossing AND overtaking (the co-course
target is then slower than own, so own overtakes). So we PRESERVE the ratio and
pick a physical own-ship speed; absolute speed is a free frame choice that does
not change the encounters. See ``_OWN_SOG_KN`` / ``_TARGET_OWN_SPEED_RATIO``.

SOURCES DISAGREE ON ABSOLUTE SCALING, NOT TOPOLOGY (documented divergence)
-------------------------------------------------------------------------
At least three independent numeric encodings of the Imazu 22 exist and do NOT
reduce to one another by a single rotate+scale:
  * CORALL / arXiv:2402.06291 (used here): [x,y] NM, own East, crossers ~3 NM.
  * Waltz & Okhrin arXiv:2211.01004 Table D.1: N/E NM, own North, all far
    targets on a 6.009 NM circle, equal speed (has a Case-8==Case-5 typo).
  * Xie et al. JMSE 12(3):372 Table A1: a third distance set, only 21 rows.
  * Lyu et al. JMSE 12(8):1289: the only per-target knots table (15 kn own).
They AGREE on the topology (which encounters, the 4/7/11 grouping) and disagree
only on absolute speed (10 / 11.7 / 14.4 / 15 kn), frame (North vs East), and
range scaling — i.e. parameter-level scaling, not the geometry itself. Per the
ticket we pick one (CORALL, because it is verbatim-verifiable runnable code) and
record the divergence here. Per-case COLREG-role tags are derived from geometry
(no source publishes an explicit role table); ``_encounter_tag`` bins by relative
course + bearing.
"""

from __future__ import annotations

import math

from .geo import marine_bearing_from_enu, wrap_deg_180
from .truth import (KNOTS_TO_MPS, ExplicitInitial, OwnShipInit, ScenarioSpec,
                    VesselSpec)

# Shared open-water datum (same origin as the sim_ms battery). The geometry is
# own-ship-relative, so the datum only fixes the WGS-84 tangent plane.
_LAT0, _LON0 = 63.45, 10.35
_OWN_MMSI = 257000000
_OWN_COG_DEG = 90.0            # own-ship heads East (matches CORALL's 0deg=+x)
# Physical own-ship speed (free frame choice); targets ride CORALL's tuned ratio.
_OWN_SOG_KN = 20.0
_TARGET_OWN_SPEED_RATIO = 18.52 / 43.3   # CORALL collision ratio ~0.4277
# Uniform range scale: 1.0 keeps CORALL's ranges verbatim. Bearings, courses and
# crossing angles are scale-invariant; CPA scales with this factor, so it is the
# knob for "keep each case short" without changing the encounter shape. 0.5 puts
# CORALL's 3-7 NM starts at a realistic close-quarters 1.5-3.5 NM so each case
# reaches CPA in ~5-9 min (vs ~14-19 min verbatim) and the battery stays cheap;
# the encounter geometry (all bearings/courses/crossing angles) is unchanged.
_RANGE_SCALE = 0.5
# Long enough to run every case through its CPA (worst ~510 s at these settings)
# plus post-CPA re-separation, which is where identity churn shows up.
_DURATION_S = 720.0
# Target MMSI block distinct from the sim_ms battery (2570001xx..2570006xx): case
# N target k -> 257010000 + N*10 + k, so fixtures never collide.
_IMAZU_MMSI_BASE = 257010000

# CORALL geometry, transcribed VERBATIM from imazu_cases.py (arithmetic kept
# as-authored so it diffs against the source). Each target: (x_nm, y_nm,
# heading_ccw_from_east_deg). Own-ship implicit at (0,0) heading 0deg=East.
_CORALL_CASES: dict[int, list[tuple[float, float, float]]] = {
    1:  [(6, 0, 180)],
    2:  [(5, -2.14, 90)],
    3:  [(3, 0, 0)],
    4:  [(3.44, 1.55 + 0.08, 295)],
    5:  [(5, -2.0 - 0.14, 90), (7 - 0.05, 0, 180)],
    6:  [(3.4, -1.5 + 0.03, 45), (3, -0.35 - 0.04, 10)],
    7:  [(3, 0, 0), (3.4, -1.5 + 0.01, 45)],
    8:  [(5, -2.13, 90), (7, 0, 180)],
    9:  [(3.4, -1.5 + 0.03, 45), (5, -2.1 - 0.05, 90)],
    10: [(3, 0.35, 350), (4.4, -2.1 + 0.20, 90)],
    11: [(5, 2.1, -90), (3.4, -1.5, 45)],
    12: [(7, 0, 180), (3, 0.3 + 0.05, -10), (3.44, -1.55 + 0.05, 45)],
    13: [(6, 0, 180), (3, 0.3 + 0.05, 350), (3.4, 1.5 + 0.05, 295)],
    14: [(3.4, -1.5, 45), (3, -0.4, 10), (5, -2.1 - 0.05, 90)],
    15: [(3, 0, 0), (3.4, -1.5, 45), (5, -2.1 - 0.05, 90)],
    16: [(3.4, 1.5 - 0.03, -45), (5, 2.1 + 0.04, -90), (5, -2.1 + -0.05, 90)],
    17: [(3, 0, 0), (3, 0.3 + 0.05, -10), (3.4, -1.5, 45)],
    18: [(3.3, -0.3 - 0.1, 10), (3.4, -1.5 + 0.05, 45), (6.5, -1.5, 135)],
    19: [(3, -0.3 - 0.07, 10), (3, 0.3 + 0.05, -10), (6.5, -1.5 - 0.03, 135)],
    20: [(3, 0, 0), (3, -0.3 - 0.05, 10), (4.4, -2.1 + 0.25, 90)],
    21: [(3 - 0.3, -0.3 - 0.05, 10), (3 - 0.3, 0.3 + 0.02, -10), (4.4, -1.9, 90)],
    22: [(3, 0, 0), (3.94, -1.6 - 0.13, 45), (5, -2.01 - 0.15, 90)],
}


def _own(sog_kn: float = _OWN_SOG_KN) -> OwnShipInit:
    return OwnShipInit(lat_deg=_LAT0, lon_deg=_LON0,
                       sog_mps=sog_kn * KNOTS_TO_MPS, cog_deg=_OWN_COG_DEG,
                       mmsi=_OWN_MMSI)


def _to_initial(x_nm: float, y_nm: float, heading_ccw_deg: float) -> ExplicitInitial:
    """CORALL (x_nm, y_nm, heading_ccw_from_east) -> ExplicitInitial.

    CORALL x=East, y=North => ENU (E, N) = (x, y) NM. Own-ship heads marine 090
    (East), so the relative bearing off the bow is (true bearing - 90). The
    target's CCW-from-East heading becomes marine course (90 - heading)."""
    e_nm = x_nm * _RANGE_SCALE
    n_nm = y_nm * _RANGE_SCALE
    range_nm = math.hypot(e_nm, n_nm)
    true_bearing = marine_bearing_from_enu(e_nm, n_nm)          # atan2(E, N), marine
    rel_bearing = wrap_deg_180(true_bearing - _OWN_COG_DEG)
    course = (90.0 - heading_ccw_deg) % 360.0
    speed = _OWN_SOG_KN * _TARGET_OWN_SPEED_RATIO
    return ExplicitInitial(rel_bearing_deg=rel_bearing, range_nm=range_nm,
                           course_deg=course, speed_kn=speed)


def _encounter_tag(rel_bearing_deg: float, rel_course_deg: float) -> str:
    """Coarse COLREG role of own-ship vs one target, from relative course +
    bearing. rel_course 0 = same heading, 180 = reciprocal. Bearing >0 =
    target on own's starboard bow."""
    c = abs(wrap_deg_180(rel_course_deg))
    if c >= 157.5:
        return "head-on"
    if c <= 22.5:
        return "overtaking"          # co-course; target is slower => own overtakes
    return "crossing-give-way" if rel_bearing_deg > 0.0 else "crossing-stand-on"


def _make_case(num: int):
    """Return a factory ``fn(seed) -> ScenarioSpec`` for Imazu case ``num``."""
    raw = _CORALL_CASES[num]

    def factory(seed: int) -> ScenarioSpec:
        vessels = []
        for k, (x_nm, y_nm, hdg) in enumerate(raw, start=1):
            init = _to_initial(x_nm, y_nm, hdg)
            rel_course = wrap_deg_180(init.course_deg - _OWN_COG_DEG)
            tag = _encounter_tag(init.rel_bearing_deg, rel_course)
            vessels.append(VesselSpec(
                encounter=tag,
                mmsi=_IMAZU_MMSI_BASE + num * 10 + k,
                name=f"IMAZU{num:02d}_T{k}",
                motion="cv",
                initial=init))
        return ScenarioSpec(name=f"imazu_{num:02d}", seed=seed, own=_own(),
                            vessels=vessels, duration_s=_DURATION_S)

    return factory


IMAZU_BATTERY = [_make_case(n) for n in sorted(_CORALL_CASES)]


def all_specs(seed: int) -> list[ScenarioSpec]:
    return [make(seed) for make in IMAZU_BATTERY]
