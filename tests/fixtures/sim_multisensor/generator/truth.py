"""Layer 1 — truth trajectories.

COLREG encounter *geometry* comes from DNV trafficgen (seeded at the wrapper
level via ``random.seed`` — trafficgen uses stdlib ``random`` module functions,
verified deterministic). trafficgen only gives clean initial conditions and
constant-velocity straight legs, which are *model-matched* to the tracker's IMM
CV mode. So the anti-model-matched-optimism dynamics the ticket mandates
(rudder-rate-limited turns with speed loss; anchored watch-circle jitter) are
synthesised here in numpy and deliberately live OUTSIDE the CV/CT model set:

* CV               -> matches IMM CV exactly (cooperative baseline legs).
* coordinated turn -> matches IMM CT (constant rate). We DON'T use this.
* rudder-limited   -> time-varying turn rate (trapezoidal ROT profile) with
  turn-coupled speed loss. Neither CV nor CT can represent it -> honest gate.

All angles are marine (deg, N=0 CW); ENU is metres (E, N). Speeds are m/s.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from .geo import Datum, enu_from_marine

KNOTS_TO_MPS = 0.514444
MPS_TO_KNOTS = 1.0 / KNOTS_TO_MPS
NM_TO_M = 1852.0


# --- specs -------------------------------------------------------------------
@dataclass
class OwnShipInit:
    lat_deg: float
    lon_deg: float
    sog_mps: float
    cog_deg: float
    mmsi: int = 257000000


@dataclass
class ExplicitInitial:
    """Deterministic placement of a target relative to own-ship, bypassing
    trafficgen. Used by fixed-geometry families (the Imazu 22) that must
    reproduce a *specific* published encounter, not a seeded random one of a
    given COLREG type.

    Marine convention throughout: bearings are relative to own-ship's bow,
    degrees, clockwise positive (so with own-ship heading north the relative
    bearing equals the compass bearing). Range is from own-ship at t=0.
    ``course_deg`` is the target's TRUE course (marine deg, N=0 CW); the target
    then runs constant-velocity (Imazu targets do not manoeuvre)."""

    rel_bearing_deg: float   # target bearing off own-ship bow (+starboard), t=0
    range_nm: float          # initial range from own-ship (nautical miles)
    course_deg: float        # target true course (marine deg, N=0 CW)
    speed_kn: float          # target speed over ground (knots)


@dataclass
class VesselSpec:
    """One target vessel: how trafficgen should place it + how it moves + which
    sensors observe it."""

    encounter: str                 # trafficgen desired_encounter_type
    mmsi: int
    name: str
    motion: str = "cv"             # "cv" | "maneuver" | "anchored"
    vector_time_min: float = 5.0
    # explicit deterministic placement (fixed-geometry families, e.g. Imazu).
    # When set, trafficgen is bypassed for the whole scenario. None => the
    # trafficgen path (the original 6-scenario battery) is used unchanged.
    initial: ExplicitInitial | None = None
    # maneuver params (motion == "maneuver")
    turn_start_s: float = 90.0
    turn_total_deg: float = 70.0   # signed; +starboard
    rot_max_dps: float = 3.0       # peak rate of turn (deg/s)
    rot_ramp_dps2: float = 0.5     # how fast ROT itself ramps (deg/s per s)
    speed_loss_frac: float = 0.25  # fractional SOG drop at peak turn
    # anchored params (motion == "anchored")
    watch_circle_m: float = 18.0
    # sensor visibility (scripted)
    radar: bool = True
    ais: bool = True
    camera: bool = False


@dataclass
class ScenarioSpec:
    name: str
    own: OwnShipInit
    vessels: list[VesselSpec]
    duration_s: float = 600.0
    base_dt_s: float = 0.25
    datum_note: str = ""
    # trafficgen encounter tuning (maritime units, as in encounter_settings.json)
    max_meeting_distance_nm: float = 0.35  # >0 => realistic non-zero CPA spread
    situation_length_min: float = 30.0
    # per-sensor / fault knobs consumed by Layer 2 (see sensors.py)
    ais_dropout: tuple[int, float, float] | None = None   # (mmsi, t0, t1)
    ownship_stale_gap: tuple[float, float] | None = None  # (t0, t1)
    ownship_heading_fault: tuple[float, float, float] | None = None  # (t0,t1,bias_deg)
    radar_range_dependent_pd: bool = False
    clutter_model: str = "poisson"        # "poisson" | "compound_k"
    clutter_burst: tuple[float, float, float, float] | None = None  # (t0,t1,e,n)
    seed: int = 0


# --- truth track -------------------------------------------------------------
@dataclass
class TruthTrack:
    vessel_id: int
    mmsi: int
    name: str
    is_ownship: bool
    t: np.ndarray            # fine time grid (s, epoch-relative to 0)
    e: np.ndarray            # ENU east (m)
    n: np.ndarray            # ENU north (m)
    sog: np.ndarray          # m/s
    cog: np.ndarray          # marine deg
    heading: np.ndarray      # marine deg
    nav_status: int          # AIS nav status (0 underway, 1 at anchor)
    radar: bool = True
    ais: bool = True
    camera: bool = False
    meta: dict = field(default_factory=dict)

    def sample(self, tq: float) -> dict:
        """Linear interpolation of state at query time ``tq`` (clamped)."""
        e = float(np.interp(tq, self.t, self.e))
        n = float(np.interp(tq, self.t, self.n))
        sog = float(np.interp(tq, self.t, self.sog))
        # interpolate angles via unit-vector to avoid wrap discontinuities
        cog = _interp_angle(tq, self.t, self.cog)
        hdg = _interp_angle(tq, self.t, self.heading)
        return {"e": e, "n": n, "sog": sog, "cog": cog, "heading": hdg}


def _interp_angle(tq: float, t: np.ndarray, ang_deg: np.ndarray) -> float:
    s = np.interp(tq, t, np.sin(np.radians(ang_deg)))
    c = np.interp(tq, t, np.cos(np.radians(ang_deg)))
    return math.degrees(math.atan2(s, c)) % 360.0


# --- trafficgen wrapper ------------------------------------------------------
def _trafficgen_target_initials(spec: ScenarioSpec) -> list[dict]:
    """Run seeded trafficgen; return one initial-condition dict per requested
    encounter, in request order. Converts SI/radians output back to marine
    deg + m/s. Raises loudly if trafficgen fails to materialise an encounter
    (a stop-and-report trigger in the ticket)."""
    # Imported lazily so the module imports even without trafficgen present
    # (e.g. for docs tooling); generation obviously requires it.
    from trafficgen.ship_traffic_generator import generate_traffic_situations
    from trafficgen.types import (
        AisNavStatus, Dimensions, Encounter, EncounterSettings, GeoPosition,
        Initial, OwnShipInitial, ShipStatic, SituationInput,
    )

    random.seed(spec.seed)

    own_static = ShipStatic(id=0, mmsi=spec.own.mmsi, name="OWN", sog_max=30.0,
                            dimensions=Dimensions(a=50, b=50, c=10, d=10))
    tgt_static = [
        ShipStatic(id=i + 1, mmsi=v.mmsi, name=v.name, sog_max=30.0,
                   dimensions=Dimensions(a=40, b=40, c=8, d=8))
        for i, v in enumerate(spec.vessels)
    ]
    own = OwnShipInitial(initial=Initial(
        position=GeoPosition(lat=spec.own.lat_deg, lon=spec.own.lon_deg),
        sog=spec.own.sog_mps * MPS_TO_KNOTS, cog=spec.own.cog_deg,
        heading=spec.own.cog_deg, nav_status=AisNavStatus.UNDER_WAY_USING_ENGINE))
    encounters = [Encounter(desired_encounter_type=v.encounter,
                            vector_time=v.vector_time_min) for v in spec.vessels]
    situation = SituationInput(title=spec.name, description=spec.name,
                               num_situations=1, own_ship=own, encounters=encounters)

    import json
    with open(_bundled_settings_path()) as f:
        settings_raw = json.load(f)
    # Tune so encounters develop within the scenario window with a realistic
    # (non-zero) CPA rather than the bundled collision course (CPA=0). These are
    # maritime units; generate_traffic_situations converts to SI internally.
    settings_raw["maxMeetingDistance"] = spec.max_meeting_distance_nm
    settings_raw["situationLength"] = spec.situation_length_min
    settings = EncounterSettings.model_validate(settings_raw)

    sits = generate_traffic_situations(situation, own_static, tgt_static, settings)
    if not sits:
        raise RuntimeError(f"trafficgen produced no situation for {spec.name!r}")
    ts = sits[0].target_ships
    if len(ts) != len(spec.vessels):
        raise RuntimeError(
            f"[{spec.name}] trafficgen materialised {len(ts)}/{len(spec.vessels)} "
            f"encounters (geometry not found). Stop-and-report per ticket.")

    out = []
    for tship in ts:
        ini = tship.initial
        out.append({
            "lat_deg": math.degrees(ini.position.lat),
            "lon_deg": math.degrees(ini.position.lon),
            "sog_mps": float(ini.sog),                       # already SI (m/s)
            "cog_deg": math.degrees(ini.cog) % 360.0,
        })
    return out


def _bundled_settings_path() -> str:
    import trafficgen
    import os
    return os.path.join(os.path.dirname(trafficgen.__file__), "settings",
                        "encounter_settings.json")


# --- explicit-geometry wrapper (fixed-geometry families) ---------------------
_MIN_SPAWN_SEP_M = 200.0   # closer than this at t=0 is a degenerate spawn


def _explicit_target_initials(spec: ScenarioSpec) -> list[dict]:
    """Place each target from its ``ExplicitInitial`` (no trafficgen, no RNG).

    Own-ship sits at the ENU origin heading ``spec.own.cog_deg``. A target at
    relative bearing ``b`` (off the bow) and range ``r`` therefore lands at
    true bearing ``own.cog + b`` and range ``r`` in the shared ENU frame. We
    return the same ``{lat_deg, lon_deg, sog_mps, cog_deg}`` dict shape as the
    trafficgen wrapper so ``build_truth`` is byte-for-byte identical downstream.

    Guards against degenerate spawns (vessels overlapping own-ship or each
    other at t=0) — a stop-and-report trigger in the Imazu ticket.
    """
    datum = Datum(spec.own.lat_deg, spec.own.lon_deg)
    enu0: list[tuple[float, float]] = [(0.0, 0.0)]   # own-ship at origin
    out = []
    for v in spec.vessels:
        ini = v.initial
        if ini is None:
            raise ValueError(
                f"[{spec.name}] vessel {v.name!r} has no ExplicitInitial")
        true_brg = (spec.own.cog_deg + ini.rel_bearing_deg) % 360.0
        e0, n0 = enu_from_marine(ini.range_nm * NM_TO_M, true_brg)
        enu0.append((e0, n0))
        lat, lon = datum.enu_to_lonlat(e0, n0)
        out.append({
            "lat_deg": lat, "lon_deg": lon,
            "sog_mps": ini.speed_kn * KNOTS_TO_MPS,
            "cog_deg": ini.course_deg % 360.0,
        })
    # degeneracy check: every pair (own + targets) must be separated at t=0.
    labels = ["OWN"] + [v.name for v in spec.vessels]
    for i in range(len(enu0)):
        for j in range(i + 1, len(enu0)):
            d = math.hypot(enu0[i][0] - enu0[j][0], enu0[i][1] - enu0[j][1])
            if d < _MIN_SPAWN_SEP_M:
                raise RuntimeError(
                    f"[{spec.name}] degenerate spawn: {labels[i]} and "
                    f"{labels[j]} are {d:.1f} m apart at t=0 "
                    f"(< {_MIN_SPAWN_SEP_M:.0f} m). Stop-and-report per ticket.")
    return out


def _target_initials(spec: ScenarioSpec) -> list[dict]:
    """Dispatch to explicit-geometry or trafficgen placement. A scenario is
    all-explicit or all-trafficgen; mixing is rejected."""
    explicit = [v.initial is not None for v in spec.vessels]
    if all(explicit):
        return _explicit_target_initials(spec)
    if not any(explicit):
        return _trafficgen_target_initials(spec)
    raise ValueError(
        f"[{spec.name}] mixed explicit/trafficgen placement is not supported")


# --- propagation kernels -----------------------------------------------------
def _propagate_cv(e0, n0, sog, cog_deg, t):
    de, dn = enu_from_marine(sog, cog_deg)          # velocity components (m/s)
    e = e0 + de * t
    n = n0 + dn * t
    return e, n, np.full_like(t, sog), np.full_like(t, cog_deg)


def _propagate_maneuver(e0, n0, sog0, cog0_deg, t, v: VesselSpec):
    """Rudder-rate-limited turn with turn-coupled speed loss.

    ROT (deg/s) follows a trapezoid: ramp 0 -> rot_max at rot_ramp (deg/s^2),
    hold until the accumulated heading change reaches |turn_total|, then ramp
    back to 0. Speed drops by up to speed_loss_frac in proportion to how hard
    we're turning. Time-varying ROT + speed => outside CV and CT."""
    dt = float(t[1] - t[0])
    n_steps = len(t)
    e = np.empty(n_steps); nn = np.empty(n_steps)
    sog = np.empty(n_steps); cog = np.empty(n_steps)
    sign = 1.0 if v.turn_total_deg >= 0 else -1.0
    target = abs(v.turn_total_deg)
    rot = 0.0                     # current rate of turn (deg/s), magnitude
    turned = 0.0                  # accumulated |heading change| (deg)
    cur_e, cur_n, cur_cog = e0, n0, cog0_deg
    ramping_down = False
    for i, ti in enumerate(t):
        turning = ti >= v.turn_start_s and turned < target - 1e-9
        if turning:
            # distance (in deg) needed to bleed ROT to zero at ramp rate
            stop_dist = (rot * rot) / (2.0 * max(v.rot_ramp_dps2, 1e-6))
            if turned + stop_dist >= target:
                ramping_down = True
            if ramping_down:
                rot = max(0.0, rot - v.rot_ramp_dps2 * dt)
            else:
                rot = min(v.rot_max_dps, rot + v.rot_ramp_dps2 * dt)
        else:
            rot = 0.0
        # turn-coupled speed loss
        speed = sog0 * (1.0 - v.speed_loss_frac * (rot / max(v.rot_max_dps, 1e-9)))
        e[i], nn[i], sog[i], cog[i] = cur_e, cur_n, speed, cur_cog % 360.0
        de, dn = enu_from_marine(speed, cur_cog)
        cur_e += de * dt
        cur_n += dn * dt
        d_head = sign * rot * dt
        cur_cog += d_head
        turned += abs(d_head)
    return e, nn, sog, cog


def _propagate_anchored(e0, n0, sog0, cog0_deg, t, v: VesselSpec, rng: np.random.Generator):
    """Watch-circle swing: the vessel rides its anchor rode, swinging slowly
    with wind/tide. Modelled as a sum of a few slow sinusoids (periods of
    minutes) — smooth and analytically differentiable, so SOG stays realistically
    small (a fraction of a knot) rather than the velocity spikes a white-noise
    walk produces. nav_status = at-anchor; never perfectly still."""
    # 3 slow components per axis; periods 90-300 s; amplitudes sum <= watch circle.
    n_comp = 3
    periods = rng.uniform(90.0, 300.0, size=(2, n_comp))
    phases = rng.uniform(0.0, 2 * math.pi, size=(2, n_comp))
    amps = rng.uniform(0.4, 1.0, size=(2, n_comp))
    amps *= (v.watch_circle_m / n_comp) / amps.mean(axis=1, keepdims=True)
    w = 2 * math.pi / periods                          # rad/s
    e = np.full_like(t, e0); n = np.full_like(t, n0)
    ve = np.zeros_like(t); vn = np.zeros_like(t)
    for k in range(n_comp):
        e += amps[0, k] * np.sin(w[0, k] * t + phases[0, k])
        n += amps[1, k] * np.sin(w[1, k] * t + phases[1, k])
        ve += amps[0, k] * w[0, k] * np.cos(w[0, k] * t + phases[0, k])
        vn += amps[1, k] * w[1, k] * np.cos(w[1, k] * t + phases[1, k])
    sog = np.hypot(ve, vn)
    cog = (np.degrees(np.arctan2(ve, vn))) % 360.0
    return e, n, sog, cog


# --- build all truth tracks for a scenario -----------------------------------
def build_truth(spec: ScenarioSpec) -> tuple[TruthTrack, list[TruthTrack]]:
    """Returns (ownship_track, target_tracks)."""
    datum = Datum(spec.own.lat_deg, spec.own.lon_deg)
    t = np.arange(0.0, spec.duration_s + spec.base_dt_s * 0.5, spec.base_dt_s)

    # own-ship: CV from its initial (encounters were designed around this).
    oe, on, osog, ocog = _propagate_cv(0.0, 0.0, spec.own.sog_mps, spec.own.cog_deg, t)
    own_track = TruthTrack(
        vessel_id=0, mmsi=spec.own.mmsi, name="OWN", is_ownship=True,
        t=t, e=oe, n=on, sog=osog, cog=ocog, heading=ocog.copy(), nav_status=0)

    initials = _target_initials(spec)
    targets: list[TruthTrack] = []
    for idx, (v, ini) in enumerate(zip(spec.vessels, initials), start=1):
        enu0 = datum.to_enu(ini["lat_deg"], ini["lon_deg"])
        e0, n0 = float(enu0[0]), float(enu0[1])
        sog0, cog0 = ini["sog_mps"], ini["cog_deg"]
        # per-vessel RNG stream (deterministic, independent per vessel).
        rng = np.random.default_rng([spec.seed, 0xA5, idx])
        if v.motion == "cv":
            e, n, sog, cog = _propagate_cv(e0, n0, sog0, cog0, t)
            nav = 0
        elif v.motion == "maneuver":
            e, n, sog, cog = _propagate_maneuver(e0, n0, sog0, cog0, t, v)
            nav = 0
        elif v.motion == "anchored":
            e, n, sog, cog = _propagate_anchored(e0, n0, sog0, cog0, t, v, rng)
            nav = 1
        else:
            raise ValueError(f"unknown motion {v.motion!r}")
        targets.append(TruthTrack(
            vessel_id=idx, mmsi=v.mmsi, name=v.name, is_ownship=False,
            t=t, e=e, n=n, sog=sog, cog=cog, heading=cog.copy(), nav_status=nav,
            radar=v.radar, ais=v.ais, camera=v.camera,
            meta={"encounter": v.encounter, "motion": v.motion}))
    return own_track, targets
