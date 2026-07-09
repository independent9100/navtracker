# Corroboration-veto isolation A/B (2026-07-09)

**Question.** The anchored-vessel corroboration veto in the occupancy layer
(`LiveOccupancyModel`: never suppress a birth within `veto_radius_m` of a recent
AIS/cooperative fix) was always-on. Increment-8 validated its *wiring* on HAXR but
could not measure its *isolated* benefit — turning on AIS engaged the veto AND
changed the fusion input at once, so the effects entangled (increment-8 eval-log:
"unmeasurable without a veto-ON/OFF toggle holding AIS constant").

**Fix under test.** A per-instance `LiveOccupancyParams::corroboration_veto_enabled`
(default `true` = shipped behaviour), ctor-threaded, no global/static state. The
default path is **byte-identical** to before (proof below). The A/B holds the AIS
arm ON in *both* runs and toggles only this flag.

## Toggle correctness + byte-identical proof

- **Fixed-input invariant** — `LiveOccupancyModel.CorroborationVetoToggleDefaultOnReproducesVetoOffFallsThrough`
  (`tests/static/test_live_occupancy_model.cpp`): with an AIS fix on a pier cell,
  default-ON drives `birthSuppression` to **exactly 0** (the veto, = pre-toggle
  behaviour); OFF returns the *same* hazard ramp as with no fix at all. Disabling
  can only RAISE suppression back to what the emitted hazards imply — never
  orphans a birth, so the ADR-0002 conservation invariant holds in BOTH states.
- **Empirical before/after** (default flag): the coverage config on
  `kattwyk_08` (AIS fed, so the veto path fires) and the philos sunset occupancy
  tests, run on the reverted (317ecfd) build vs the toggle build — **all 811 HAXR
  metric rows identical; philos hazard diagnostics identical.** Default-on is a
  no-op relative to the always-on original.

## HAXR 3-site A/B — AIS arm ON in both, veto toggled

Fixed shore stations kattwyk / parkhafen / seemannshöft, hour 08, decimated
(eps=50), common 285 s window `[29096.3, 29380.9]`. Config
`imm_cv_ct_pmbm_occupancy_detector_coverage`. Metric set = increment-8's.
Harness: `tests/benchmark/test_veto_isolation_haxr_ab.cpp` (single seed; replay).

| site | metric | veto OFF | veto ON | Δ (ON−OFF) |
|---|---|---:|---:|---:|
| **kattwyk** | occ_suppress_hits | 45848 | 39417 | **−6431 (−14.0%)** |
| | card_err_mean | −0.38736 | −0.38258 | +0.00478 |
| | gospa_mean | 34.1657 | 34.1706 | +0.0049 |
| | gospa_missed | 831.124 | 831.067 | −0.056 |
| | gospa_false | 753.652 | 754.551 | +0.899 |
| | lifetime_ratio | 0.0071625 | 0.0071625 | 0 |
| | occ_peak_structures | 48 | 48 | 0 |
| **parkhafen** | occ_suppress_hits | 24578 | 13599 | **−10979 (−44.7%)** |
| | card_err_mean | −3.71361 | −3.69496 | +0.01865 |
| | gospa_mean | 40.8262 | 40.8451 | +0.0189 |
| | gospa_missed | 1343.33 | 1343.28 | −0.056 |
| | gospa_false | 600.612 | 604.286 | +3.674 |
| | lifetime_ratio | 0.0052865 | 0.0054057 | +0.00012 |
| | occ_peak_structures | 46 | 46 | 0 |
| **seemannshöft** | occ_suppress_hits | 104810 | 69631 | **−35179 (−33.6%)** |
| | card_err_mean | −1.58887 | −0.95939 | **+0.62949** |
| | gospa_mean | 45.825 | 46.2971 | +0.4721 |
| | gospa_missed | 1519.22 | 1516.77 | −2.448 |
| | gospa_false | 1201.45 | 1324.90 | **+123.45** |
| | lifetime_ratio | 0.0077067 | 0.0105380 | +0.00283 |
| | occ_peak_structures | 76 | 76 | 0 |

Reading:
- **The veto is NOT inert.** It lifts **14–45 % of occupancy suppression hits**
  near AIS/cooperative fixes — a large, consistent, directly-attributable effect
  (disabling it can only raise suppression, so the sign is guaranteed; the
  magnitude is the measurement). `occ_peak_structures` is unchanged (the veto
  does not change what is *learned* as structure — only whether a birth is
  suppressed there), confirming the isolation is clean.
- **It is protective (ADR-0002).** Lifting suppression near known vessels
  recovers missed tracks: `card_err_mean` moves toward 0 on all three sites
  (seemannshöft +0.63), `gospa_missed` falls on all three, `lifetime_ratio` rises.
- **The cost, named.** In dense clutter, the same lift within `veto_radius_m`
  (100 m) of a fix also admits nearby phantoms: `gospa_false` rises (seemannshöft
  +123). Net accuracy (`gospa_mean`) is therefore ~flat on kattwyk/parkhafen and
  slightly worse at seemannshöft (+0.47) — the recovered-vs-admitted tracks
  roughly cancel on these fixed-shore dense-harbor stations.

## Sim anchored A/B — perfect truth

`sim_ms_anchored_camera`, config `imm_cv_ct_pmbm_occupancy_detector`
(`extended_cells_min=1`, the most permissive structure gate), veto ON vs OFF.
Harness: `tests/benchmark/test_veto_isolation_sim_ab.cpp`.

**Result: the veto is inert here — every metric is byte-identical ON vs OFF**
(`occ_peak_structures = 0`, `occ_suppress_hits = 0` in both arms;
`gospa_mean = 39.67`, `card_err = 3.20`, `lifetime_ratio = 0.60`, identical).
The occupancy layer *is* fed (`feed_occupancy = occupancy_feed_ != nullptr`,
`PmbmTracker.cpp:1700`) but forms **no structure** on open-water sim: the
anchored vessel is *tracked* (high existence → low clutter-weight into the
occupancy feed), so it never becomes suppressible structure, and there is no
fixed harbour structure in the scenario. With nothing suppressed, there is
nothing for the veto to lift. The perfect-truth *protective* demonstration the
ticket hoped for is therefore carried by the fixed-input unit test (a pier the
veto protects), not by this open-water scenario — reported honestly.

## Verdict — (a) real and protective; default UNCHANGED (ON)

The veto's isolated effect is **real and protective**: it lifts 14–45 % of
suppression near AIS/cooperative fixes and recovers missed tracks (the ADR-0002
"a known vessel must stay track-eligible" payoff, now measured). It is **kept ON**
— that is the ADR-0002-mandated, principled behaviour, and this change adds only
the ablation toggle; no default/behaviour change.

The trade to hand the arbiter (not a default decision here): on **fixed-shore
dense-harbor** stations the lifted suppression also admits phantoms within
`veto_radius_m` of a fix, so net GOSPA is flat-to-slightly-worse there while
cardinality/miss improve. If that phantom cost matters for a deployment, the lever
is `veto_radius_m` (tighten it), **not** disabling the veto — an arbiter/user call.

## Provenance

- Build base: `317ecfd` (master); branch `veto-isolation-ab`.
- Decimated 285 s window slices produced by `awk '$1<29381'` on the eps=50 dec50
  fixtures (parkhafen/seemannshöft; kattwyk `_w285` pre-existed). All local-only
  (git-ignored `tests/fixtures/haxr_cfar/out/`).
- Fixture md5 (plots / AIS / stations):
  - `kattwyk_08_dec50_w285.csv`  `304cdeb8e81f03cbddb52d629fab22a9`
  - `parkhafen_08_dec50_w285.csv`  `c0930398ada0d62eea7f92a09d8937f2`
  - `seemannshoeft_08_dec50_w285.csv`  `203f3dc630056806ae1bc1f09d1c8e19`
  - `kattwyk_08-UTC.csv`  `b518d6071e301d2ee4e0cacfd7d45099`
  - `parkhafen_08-UTC.csv`  `8fab9924ecbcd995d5635a313c176964`
  - `seemannshoeft_08-UTC.csv`  `bcdad222cbdc0527fa7a75d44d88e5f7`
  - `stations.csv`  `2a907b16b5f4bcf28509e27ed786a6fd`
- Reproduce (HAXR): `ctest --test-dir build -R VetoIsolationHaxrAB` (fixtures wired) — ~6.5 min.
- Reproduce (sim): `ctest --test-dir build -R VetoIsolationSimAB` (SIMMS fixtures wired).
- Byte-identical: `ctest --test-dir build -R CorroborationVetoToggle` + the
  reverted-vs-toggle bench diff recorded in the 2026-07-09 eval-log entry.
