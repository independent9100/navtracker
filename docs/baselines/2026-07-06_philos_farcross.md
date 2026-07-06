# Philos ais_ferry_far + almost_cross measurement pass (pre-water Tier-2 #10)

Date: 2026-07-06. Measurement-only (no default touched, no algorithm changed).
Ticket: `docs/superpowers/plans/2026-07-06-philos-farcross-measurement-ticket.md`.
Metrics CSV: `2026-07-06_philos_farcross.csv` (per clip × config × arm ×
truth-source, with a circularity-label column).

## Clip facts (read first — clip assumptions have bitten before)

| | `ais_ferry_far` | `almost_cross` |
|---|---|---|
| Duration | **19 s** (radar_plots tod 1667846215–234) | **50 s** (1635536668–718) |
| Radar plots | 1038 (loader keeps 1038) | 1510 (loader keeps 1504) |
| Ownship rows | 662 | 2964 |
| AIS | 40 rows, **20 unique MMSIs**, carries `sog_mps`+`cog_deg` | **0 rows (empty)** |
| `radar_truth.csv` | present, 40 rows | **absent** |
| Video labels | none | none |

`ais_ferry_far` is a **19-second spot check** with **40 truth samples** across
20 vessels. The accuracy number below is valuable because it is the *only*
real-data accuracy figure with honest truth we have before the water test — but
it is a spot check on an untuned clip, **not a benchmark**, and must not be
quoted as one.

## Finding 1 — `radar_truth.csv` is AIS-derived, not independent (load-bearing)

`tests/fixtures/philos/build_truth.py` (line 20: "a quick analytical
projection") builds `radar_truth.csv` by projecting each **AIS** row's lat/lon
into the radar's `(range_m, azimuth)` frame, `uid = MMSI`. Verified against the
data: `radar_truth`'s 20 uids are **exactly** the 20 AIS MMSIs, sampled at the
AIS timestamps (100 % overlap). So `philos_radartruth` is **AIS truth in the
radar frame, not independent of AIS**. Consequences under the circularity rule:

- The `philos` vs `philos_radartruth` comparison is **one truth (AIS) in two
  frames** — a projection/datum consistency check, **not** two independent
  truth sources.
- Any **AIS-consuming** arm scored against `radar_truth` is **still circular**.

The code comment claiming independence was corrected, a corrective eval-log
entry filed, and past usage audited (all clean — see the eval-log entry).

## Finding 2 — no radar-only measurement arm existed (fixed, bit-identical)

The philos scenario fed AIS as *measurements* unconditionally (`truth_source`
only switched the truth). So without a change there was **no honest accuracy
number possible** for this clip (radar+AIS vs AIS-derived truth = circular).
Added an env-gated **radar-only measurement mode** (`PHILOS_RADAR_ONLY`,
default off = today's radar+AIS, proven bit-identical) so a radar-only arm can
be scored against AIS truth honestly. Also added `PHILOS_CLIP` to select the
clip (default `ais_ferry_near`, bit-identical). Both proven byte-identical
before/after and unset-vs-explicit-default.

## Finding 3 — the #20 SOG/COG velocity path is unreachable via replay

The `ais.csv` carries `sog_mps`/`cog_deg` (row-level: e.g. MMSI 367782940 SOG
12.6, COG 312.7), so this *would* be the first real-data exercise of the #20
increment-2 velocity path. But the replay loader
`adapters/replay/AisCsvReplayAdapter::loadAisCsv` parses only time/mmsi/lat/lon
and emits **`Position2D` only** — the SOG/COG columns are ignored, and there is
no nav-status column at all. The #20 velocity path lives in
`adapters/ais/AisAdapter` (the NMEA path), not the replay path. **R11 identity
does flow** (`m.hints.mmsi` is set). Wiring `loadAisCsv` to emit
`PositionVelocity2D` from SOG/COG would change existing philos bench numbers
(not bit-identical) → an arbiter change with its own A/B, **not** a
measurement-only one. Listed below; not done.

## Arm A — `ais_ferry_far` radar-only accuracy (HONEST)

Radar-only arm (does not consume AIS) scored vs AIS-derived truth in both
frames. Cutoff c = 20 m, p = α = 2. Single seed (replay, deterministic).

| config | truth frame | gospa | loc | missed | false | card_err | lifetime | pos_rmse |
|---|---|--:|--:|--:|--:|--:|--:|--:|
| mht | AIS-ENU | 42.34 | 0 | 1652.6 | 231.6 | −7.1 | 0.00 | — |
| mht | AIS-range/az | 42.34 | 0 | 1652.6 | 231.6 | −7.1 | 0.00 | — |
| pmbm_coverage_land | AIS-ENU | 81.68 | 35.57 | 1631.6 | 5305.3 | +18.4 | 0.10 | 46.0 |
| pmbm_coverage_land | AIS-range/az | 81.70 | 38.50 | 1631.6 | 5305.3 | +18.4 | 0.10 | 46.2 |
| pmbm_adapt | AIS-ENU | 78.02 | 0 | 1652.6 | 4484.2 | +14.2 | 0.05 | 54.1 |
| pmbm_adapt | AIS-range/az | 78.02 | 0 | 1652.6 | 4484.2 | +14.2 | 0.05 | 54.7 |

**Honest read (spot check, 19 s / 40 truth rows):** radar-only tracking on this
clip is **missed-dominated** — 19 seconds is barely a confirmation window, so
little is established against the 20-vessel AIS set. MHT stays conservative and
confirms essentially **no** matched track (loc 0, lifetime 0, card −7): its low
gospa is degenerate (under-production dodges the false penalty), not accuracy.
PMBM actually forms tracks (lifetime 0.05–0.10, pos_rmse **46–54 m**) but its
gospa is dominated by `false` (5305 / 4484) — largely radar returns (structure,
non-AIS craft) that the sparse AIS truth **cannot score**, the same
truth-incompleteness caveat the R8 entries flagged. **The only real accuracy
signal is PMBM's ~46–54 m position error where tracks do match**; treat even
that as a spot check. No config is "good" here; the gospa ranking is a
missed/false trade-off against an incomplete truth, not a quality ordering.

### Two-frames consistency check (Finding 1 relabel) — PASS

The AIS-ENU vs AIS-range/az frames agree to within projection precision:
gospa identical to 5 s.f. (mht 42.34 ≡ 42.34), the only movement in the `loc`
term (coverage_land 35.57 vs 38.50, ~8 %; adapt/mht loc 0 both frames). A
disagreement this small is `build_truth.py`'s flat-earth projection vs the
tracker's ENU geodesy — a consistency signal, **not** a truth dispute. If this
delta ever grows, it is a bug in `build_truth.py` or our geodesy.

## Arm B — `ais_ferry_far` radar+AIS (CIRCULAR → mechanics only)

Arm consumes AIS; truth is AIS-derived ⇒ accuracy is circular. Reported as
mechanics.

| config | gospa | card_err | lifetime | false | id_switches | pos_rmse |
|---|--:|--:|--:|--:|--:|--:|
| mht | 49.20 | −1.6 | 0.42 | 957.9 | 0.05 | 25.2 |
| pmbm_coverage_land | 81.58 | +18.3 | 0.15 | 5284 | 0 | 46.5 |
| pmbm_adapt | 76.56 | +17.2 | 0.53 | 4579 | 0.15 | 18.5 |

**Mechanics:** AIS fusion transforms tracking vs radar-only (Arm A) — MHT
lifetime 0.00 → 0.42, pos_rmse — → 25 m; adapt lifetime 0.05 → 0.53, pos_rmse
54 → 18.5 m. The known PMBM philos over-count persists (card +17–18, false
~5000; consistent with "over-count = static structure + diffuse clutter"). MHT
stays conservative (card −1.6). Identity is stable (id_switches ≈ 0). #20
velocity path not exercised (Finding 3).

## Arm C — `almost_cross` (NO truth: 0 AIS, no radar_truth, no labels)

The bench harness is **truth-driven** (snapshots at truth timestamps), so a
truthless clip yields **zero** bench metrics even though the tracker runs
(4.17 s over the plots). Quantified instead with the direct-tracker harness
(`tests/replay/test_philos_farcross.cpp`, EKF+GNN, fixed 1 s clock):

- radar = 1504 plots, AIS = 0. **final_tracks = 73, unique_ids = 211** over 50 s.
- **ADR-0002 anchorage/persistence canary: PASS** — the persistent radar
  returns surface as confirmed tracks that survive to end-of-clip; no contact is
  suppressed into nothing.
- Heavy raw-plot over-count (211 ids in 50 s for a near-crossing of ~1–2
  vessels + own-ship) — the known raw-radar over-count, no clutter suppression
  in this smoke harness. Mechanics-grade presence, not a cardinality claim.

ID-stability through the crossing and per-config id-switch counts are **not**
obtainable without truth; they need a truth source this clip lacks.

## Findings for the arbiter (not acted on)

1. **#20 velocity path unreachable via replay** (Finding 3). Wiring
   `loadAisCsv` to emit `PositionVelocity2D` from SOG/COG would exercise it on
   real data, but is not bit-identical (changes existing philos numbers) → needs
   its own A/B. Decision: wire it (and re-baseline philos) or leave replay
   AIS as Position2D.
2. **Bench cannot score truthless clips** (Arm C). The truth-driven snapshot
   design means `almost_cross`-class clips get no metrics. A fixed-clock
   evaluation mode (as in `test_philos_farcross`/`test_philos_ospa`) would let
   the bench report lifecycle/cardinality without truth. Decision: add a
   clockless eval mode to the bench, or keep such clips test-only.
3. **`radar_truth` naming** (Finding 1). Now correctly documented as
   AIS-in-radar-frame; if a genuinely independent truth is ever wanted it means
   hand-labeling radar/video (analyst time), not a re-projection.

## Reproduce

```bash
cmake --build build --target navtracker_bench_baseline navtracker_tests
# Arm A (radar-only, honest) — both truth frames, per config:
PHILOS_CLIP=ais_ferry_far PHILOS_RADAR_ONLY=1 \
  ./build/bench/navtracker_bench_baseline --scenario-filter philos \
  --config-eq imm_cv_ct_pmbm_adapt --seeds 1 --run-id armA --out /tmp/d10
# Arm B (radar+AIS, mechanics):
PHILOS_CLIP=ais_ferry_far ./build/bench/navtracker_bench_baseline \
  --scenario-eq philos --config-eq imm_cv_ct_pmbm_adapt --seeds 1 --run-id armB --out /tmp/d10
# Arm C + smoke/canary:
./build/navtracker_tests --gtest_filter='PhilosFarCross.*'
```

## Fixture checksums (drift-guard, per camera-bearing precedent)

```
707978ccd7ee5384bf644e431e867b50  ais_ferry_far/ownship.csv
d743cce51cbde63482a1d288c2cba148  ais_ferry_far/ais.csv
300109a4ac74aa5fabab7bd5c6a015da  ais_ferry_far/radar_plots.csv
3fdff546dac8f716cf341756231fe333  ais_ferry_far/radar_truth.csv
e78bd6031c719a596bc95a07070a057b  almost_cross/ownship.csv
b9595f9a7960316f575e28748a5a5e39  almost_cross/ais.csv   (empty: header only)
9e4a3725cb377e917f294313f4a94c59  almost_cross/radar_plots.csv
```
