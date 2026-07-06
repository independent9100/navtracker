# 2026-07-06 — Multi-sensor simulation battery (first fusion accuracy gate)

The first controlled **fusion accuracy** measurement navtracker has: a
radar+AIS(+camera) arm scored against **complete truth that is independent of
every sensor by construction**. Real-data clips (philos, HAXR) cannot give this
— no truth source there is independent of the sensors being scored (the philos
`radartruth` circularity finding). Simulation truth is the generator's ground
truth, consumed by no arm, so both radar+AIS and radar-only are HONEST against
it.

Fixtures: seeded Python generation under `tests/fixtures/sim_multisensor/`
(local-only; regenerate per that dir's README). Battery is 6 scenarios, seed 0.
Metrics: GOSPA `c=20, p=α=2` (m); OSPA cutoff 500 m; assoc gate 100 m.
Reproduce:

```
SIMMS_DIR=$PWD/tests/fixtures/sim_multisensor \
  ./build/bench/navtracker_bench_baseline --with-simms --skip-replays \
  --scenario-filter sim_ms --config-eq <config> --seeds 1 --run-id <id> --out docs/baselines/
```

Anti-model-matched-optimism is designed in (see the fixtures README): the
crossing give-way and overtaking vessels execute **rudder-rate-limited turns
with speed loss** (outside the CV/CT IMM set); `sim_ms_clutter_burst` uses a
**compound-K** clutter field (not flat Poisson). A gate that flatters the
filter's own assumptions would manufacture false confidence — these do the
opposite, and the numbers below show they bite.

## MHT (`imm_cv_ct_mht`) — radar+AIS fusion arm

| scenario | gospa | ospa | card_err | lifetime | id_sw | breaks | rmse(m) |
|---|---|---|---|---|---|---|---|
| sim_ms_headon          | 23.0 |  39.9 |  0.02 | 0.991 |  1.5 |  0.5 | 26.5 |
| sim_ms_crossing        | 28.6 |  89.1 |  0.21 | 0.966 |  7.3 |  8.3 | 24.6 |
| sim_ms_overtaking      | 22.7 |  74.5 |  0.11 | 0.936 | 12.5 | 16.0 | 25.1 |
| sim_ms_ais_dropout     | 21.9 |  33.1 |  0.03 | 0.992 |  0.5 |  1.0 | 22.2 |
| sim_ms_clutter_burst   | 30.4 | 183.8 | **2.51** | 0.992 |  3.5 |  0.5 | 26.0 |
| sim_ms_anchored_camera | 29.9 | 309.1 | -0.14 | 0.669 |  0.7 |  5.0 | 33.2 |

## PMBM (`imm_cv_ct_pmbm_coverage_land`) — radar+AIS fusion arm

| scenario | gospa | ospa | card_err | lifetime | id_sw | breaks | rmse(m) |
|---|---|---|---|---|---|---|---|
| sim_ms_headon          | 24.6 | 121.1 |  0.38 | 0.960 | 0 | 10.0 | 27.5 |
| sim_ms_crossing        | 29.1 | 100.3 |  0.33 | 0.985 | 0 |  3.7 | 24.2 |
| sim_ms_overtaking      | 23.4 | 141.9 |  0.56 | 0.992 | 0 |  1.5 | 20.2 |
| sim_ms_ais_dropout     | 23.8 | 124.7 |  0.46 | 0.993 | 0 |  1.0 | 21.8 |
| sim_ms_clutter_burst   | 33.6 | 267.9 | **3.48** | 0.995 | 0 |  0.0 | 25.7 |
| sim_ms_anchored_camera | 30.3 | 308.0 | -0.21 | 0.616 | 0 | 15.3 | 40.3 |

## Fusion vs radar-only (`imm_cv_ct_mht`) — the headline

The first controlled fusion-vs-single-sensor delta. Radar-only run adds
`SIMMS_RADAR_ONLY=1` (drops AIS + camera; same independent truth).

| scenario | fusion ospa | radar-only ospa | fusion card_err | radar-only card_err |
|---|---|---|---|---|
| sim_ms_ais_dropout     | **33.1** | 67.2 |  0.03 |  0.06 |
| sim_ms_headon          | **39.9** | 61.4 |  0.02 |  0.02 |
| sim_ms_overtaking      | **74.5** | 87.2 |  0.11 | -0.02 |
| sim_ms_crossing        | 89.1 | **73.4** |  0.21 |  0.10 |
| sim_ms_clutter_burst   | 183.8 | **127.9** |  2.51 |  0.93 |
| sim_ms_anchored_camera | 309.1 | 292.1 | -0.14 | -0.98 |

## Reading the numbers

- **Fusion helps where continuity / absolute position matter.** On
  `ais_dropout` fusion roughly halves OSPA (33 vs 67): AIS anchors identity and
  absolute position, and when it drops the radar carries the track (lifetime
  0.99, 0.5 id-switches). Head-on and overtaking also improve with AIS.
- **The compound-K instrument bites.** `sim_ms_clutter_burst` is the only
  scenario where both trackers OVER-count (MHT `card_err +2.51`, PMBM `+3.48`):
  their clutter term assumes flat Poisson, and the gamma-modulated field is
  clumpy and heavy-tailed. Radar-only over-counts less (+0.93) than fusion
  (+2.51) — the AIS-seeded tracks interact with the clutter births. This is the
  designed discrimination target for the future clutter/birth-model campaign: a
  spatially-varying-λ model should measurably beat uniform-λ here.
- **Out-of-model dynamics cost identity, not position.** `overtaking` (12.5
  id-switches, 16 breaks) and `crossing` (7.3, 8.3) — the rudder-rate-limited
  turns with speed loss are outside the CV/CT IMM set, so the filter loses and
  re-acquires identity through the maneuver while position RMSE stays ~25 m. An
  IMM with a rudder/maneuver mode should reduce the switch count here — a clean
  future A/B.
- **The camera-only contact is never invisible, but never a full track.**
  `anchored_camera` OSPA ~309 and slightly-negative card_err reflect the
  radar-silent, AIS-silent contact: bearing-only measurements cannot birth a
  track (by design — corroborate, never initiate), so it surfaces as the
  wedge/hazard channel, not a kinematic track. ADR-0002 "never suppressed into
  nothing" holds (it is present in truth throughout and produces camera
  bearings); full localization of a camera-only target is a triangulation /
  bearing-wedge concern, not this gate.
- **MHT vs PMBM.** PMBM holds identity perfectly (0 id-switches everywhere,
  its Bernoulli-track identity is structural) but shows higher OSPA and more
  breaks on several scenarios at this single seed; MHT has lower OSPA but pays
  in switches on the maneuvering scenarios. Not a verdict — a single-seed
  snapshot to expand into a multi-seed A/B.

See `docs/algorithms/evaluation-log.md` (2026-07-06) for the fixture checksums
and the honest-vs-circular framing.
