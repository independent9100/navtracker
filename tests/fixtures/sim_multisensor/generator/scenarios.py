"""The 6-scenario battery.

Each scenario is a factory ``fn(seed) -> ScenarioSpec`` so the battery can be
regenerated across multiple generation seeds. Anti-model-matched-optimism
coverage (ticket §4):

* out-of-IMM-model dynamics in >=2 scenarios: crossing give-way vessel does a
  rudder-rate-limited turn (sim_ms_crossing); overtaking vessel maneuvers
  (sim_ms_overtaking). Both are time-varying-ROT + speed-loss, outside CV/CT.
* non-Poisson clutter in >=1 scenario: sim_ms_clutter_burst uses compound-K.

Own-ship starts at a Trondheimsfjord-ish open-water datum and steams north.
"""

from __future__ import annotations

from .truth import OwnShipInit, ScenarioSpec, VesselSpec

_LAT0, _LON0 = 63.45, 10.35
_OWN_MMSI = 257000000


def _own(sog=6.0, cog=0.0):
    return OwnShipInit(lat_deg=_LAT0, lon_deg=_LON0, sog_mps=sog, cog_deg=cog,
                       mmsi=_OWN_MMSI)


def sim_ms_crossing(seed: int) -> ScenarioSpec:
    """3-vessel crossing geometry, radar+AIS. The give-way vessel executes a
    rudder-rate-limited starboard turn with speed loss (out-of-model)."""
    return ScenarioSpec(
        name="sim_ms_crossing", seed=seed, own=_own(),
        vessels=[
            VesselSpec("crossing-give-way", 257000101, "CROSS_GW", motion="maneuver",
                       turn_start_s=180, turn_total_deg=55, rot_max_dps=2.0,
                       rot_ramp_dps2=0.4, speed_loss_frac=0.2),
            VesselSpec("crossing-stand-on", 257000102, "CROSS_SO", motion="cv"),
            VesselSpec("overtaking-stand-on", 257000103, "OVERT_SO", motion="cv"),
        ])


def sim_ms_headon(seed: int) -> ScenarioSpec:
    """Head-on pair, radar+AIS."""
    return ScenarioSpec(
        name="sim_ms_headon", seed=seed, own=_own(),
        vessels=[
            VesselSpec("head-on", 257000201, "HEADON_A", motion="cv"),
            VesselSpec("head-on", 257000202, "HEADON_B", motion="cv",
                       vector_time_min=6.0),
        ])


def sim_ms_overtaking(seed: int) -> ScenarioSpec:
    """Overtaking + one maneuvering (out-of-model) vessel, radar+AIS."""
    return ScenarioSpec(
        name="sim_ms_overtaking", seed=seed, own=_own(sog=7.0),
        vessels=[
            VesselSpec("overtaking-give-way", 257000301, "OVERT_GW", motion="maneuver",
                       turn_start_s=150, turn_total_deg=40, rot_max_dps=1.5,
                       rot_ramp_dps2=0.3, speed_loss_frac=0.3),
            VesselSpec("crossing-give-way", 257000302, "CROSS_GW", motion="cv"),
        ])


def sim_ms_ais_dropout(seed: int) -> ScenarioSpec:
    """AIS dies mid-scenario for one vessel; the track must survive on radar
    with identity retained (R11) and re-attach on AIS return."""
    return ScenarioSpec(
        name="sim_ms_ais_dropout", seed=seed, own=_own(),
        vessels=[
            VesselSpec("crossing-give-way", 257000401, "DROPOUT", motion="cv"),
            VesselSpec("head-on", 257000402, "STEADY", motion="cv"),
        ],
        ais_dropout=(257000401, 200.0, 380.0))


def sim_ms_clutter_burst(seed: int) -> ScenarioSpec:
    """Compound-K clutter field + a spatial burst. The over-count instrument:
    a spatially-varying-λ clutter model should beat uniform-λ here measurably."""
    return ScenarioSpec(
        name="sim_ms_clutter_burst", seed=seed, own=_own(),
        vessels=[
            VesselSpec("head-on", 257000501, "TGT_A", motion="cv"),
            VesselSpec("crossing-stand-on", 257000502, "TGT_B", motion="cv"),
        ],
        clutter_model="compound_k",
        clutter_burst=(120.0, 240.0, 1200.0, 900.0))


def sim_ms_anchored_camera(seed: int) -> ScenarioSpec:
    """Anchored nav_status=1 vessel (watch-circle jitter) + a camera-only
    radar-silent contact (never-invisible exercise: #17 wedge + ADR-0002)."""
    return ScenarioSpec(
        name="sim_ms_anchored_camera", seed=seed, own=_own(sog=5.0),
        vessels=[
            VesselSpec("crossing-stand-on", 257000601, "ANCHORED", motion="anchored",
                       watch_circle_m=16.0),
            # radar-silent, camera-visible: scripted Pd=0 on radar.
            VesselSpec("head-on", 257000602, "CAM_ONLY", motion="cv",
                       radar=False, ais=False, camera=True),
            VesselSpec("overtaking-stand-on", 257000603, "COOP", motion="cv"),
        ])


BATTERY = [
    sim_ms_crossing, sim_ms_headon, sim_ms_overtaking,
    sim_ms_ais_dropout, sim_ms_clutter_burst, sim_ms_anchored_camera,
]


def all_specs(seed: int) -> list[ScenarioSpec]:
    return [make(seed) for make in BATTERY]
