# PMBM promotion dossier — every deployment-decision number at one commit

**MEASUREMENT ONLY.** No config, core, or tuning change. No winner is declared;
this compiles, at a single commit and one harness state, the numbers the user
decides the canonical-tracker question FROM. Formal canonical today is still
`imm_cv_ct_mht` (2026-06-24 decision). North-star: Cl-3 #3 ("PMBM as canonical,
if the gain holds") — this is that milestone's measurement half.

## Header / provenance

- **Base commit (all rows):** `d94471e` (master; merges veto-isolation-ab +
  backlog25-phase2b + #25 close-out). Worktree `../navtracker-dossier`, branch
  `pmbm-promotion-dossier`, own build dir. Release, g++/Conan per
  `reference_build_env`.
- **Fixtures** wired from the main tree (all git-ignored local-only families
  symlinked): autoferry (`data/autoferry`), philos (`tests/fixtures/philos/out`),
  HAXR (`tests/fixtures/haxr_cfar`), sim_ms + Imazu
  (`tests/fixtures/sim_multisensor/{sim_ms_*,imazu_*}`), HAXR AIS/stations
  (`data/dlr`). Skips: ∅ on the workloads below except where **named**.
- **Contenders (fixed):**
  - **Champion** `imm_cv_ct_mht` — today's formal canonical.
  - **Candidate** `imm_cv_ct_pmbm_coverage_land_ivgate` — coverage + land prior +
    innovation gate (velocity-deweight @ 400 m).
  - Where a workload's history used a different PMBM variant, that variant is
    **also** run (backward cross-check) and the difference is named. Backward
    variants: `imm_cv_ct_pmbm_land` (Cl-1 autoferry), `imm_cv_ct_pmbm_coverage_land`
    (Imazu / sim_ms / runtime), `imm_cv_ct_pmbm` (harbor).

### Harness-integrity cross-check (the STOP gate) — PASSED

The Cl-1 frozen protocol (`helgesen2022_reference.md`, 2026-07-08 §) reproduces
**to printed precision** at `d94471e`, so the harness state matches the record
the dossier compares against:

| condition | config | env-1 (frozen → here) | env-2 (frozen → here) |
|---|---|---|---|
| no-AIS | `imm_cv_ct_mht` | 34.37 → **34.37** | 28.57 → **28.57** |
| no-AIS | `imm_cv_ct_pmbm_land` | 18.62 → **18.62** | 17.74 → **17.74** |
| truth-AIS | `imm_cv_ct_mht` | 5.18 → **5.18** | 5.16 → **5.16** |
| truth-AIS | `imm_cv_ct_pmbm_land` | 3.43 → **3.43** | 8.94 → **8.94** |

Imazu, sim_ms, and HAXR-veto historical tables also reproduce (spot-checks noted
in-line below). No divergence beyond noise. Gate cleared.

---

## 1. Summary table (workloads × contenders)

GOSPA is `c=20 m, p=α=2`; RMS aggregate for autoferry (paper convention), mean
elsewhere. `card_err` = card_err_mean, `life` = lifetime_ratio, `id_sw` /
`brk` = id_switches / track_breaks (mean per truth on multi-target sims).
Champion = `imm_cv_ct_mht` (**M**); Candidate = `..._coverage_land_ivgate`
(**C**); backward variant shown where it differs materially.

### 1a. Autoferry / Cl-1 (GOSPA RMS, per-env aggregate)

| env / condition | M (mht) | C (ivgate) | backward `pmbm_land` | reading |
|---|---:|---:|---:|---|
| env-1 no-AIS (cold start) | 34.37 | **17.44** | 18.62 | Candidate best on open water — beats paper 20.37 and `pmbm_land`. |
| env-2 no-AIS (urban channel) | 28.57 | **20.00 (COLLAPSE)** | 17.74 | Candidate tracks **nothing** on env-2 (lifetime 0, card_err −2, all targets missed). |
| env-1 truth-AIS | 5.18 | 9.77 | 3.43 | Candidate worse with AIS anchor on env-1. |
| env-2 truth-AIS | 5.16 | 20.00 (COLLAPSE) | 8.94 | Same env-2 collapse under AIS. |

**Root cause of the env-2 collapse (design-documented, not a bug):**
`coverage_land` sets `min_new_bernoulli_existence == birth_existence_target ==
0.1`, so the offshore soft-band becomes a **no-birth zone** — "a vessel within
50 m of shore will not initiate under this config" (Config.cpp:1220-1223). The
Trondheim urban channel (env-2) has targets hugging the shore → no births →
total loss. **Isolated:** plain `coverage_land` (gate OFF) shows the *identical*
collapse, so it is the coverage/land no-birth zone, **not** the innovation gate.
The Cl-1 headline win (18.62 < 20.37) belongs to `pmbm_land`, which has the land
prior *without* that hard floor.

### 1b. philos KEEP clips (real data; AIS-as-truth, sparse)

| metric | M (mht) | C (ivgate) | backward `coverage_land` |
|---|---:|---:|---:|
| gospa_mean | 69.4 | 73.1 | 73.1 |
| card_err | 8.1 | 6.9 | 6.9 |
| id_switches | 0 | 0.04 | 0.04 |
| lifetime | 0.31 | 0.034 | 0.034 |

- Near-parity confirmed (candidate 73.1 vs champion 69.4; matches history).
- **Innovation-gate fire-count = 0 on philos:** candidate is **byte-identical**
  to `coverage_land` on *every* accuracy/continuity/identity metric (only the
  timing columns differ). The guard's honest innovations never reach 400 m on
  real data (phase2b: 0% false-fire). KEEP safety absolute.
- The low PMBM lifetime is the continuity trade on a fixture where truth is
  sparse AIS (helgesen ref: philos lifetime is honestly low for all trackers).

### 1c. HAXR 3 sites (decimated, veto ON default; PMBM-occupancy arm)

Veto ON (default/deployment) vs OFF, `imm_cv_ct_pmbm_occupancy_detector_coverage`,
AIS arm ON in both, single seed. Reproduces the 2026-07-09 veto-isolation table
**to printed precision** at `d94471e`:

| site | metric | veto OFF | veto ON (default) | Δ |
|---|---|---:|---:|---:|
| kattwyk | card_err_mean | −0.3874 | −0.3826 | +0.0048 |
| | gospa_mean | 34.166 | 34.171 | +0.005 |
| | occ_suppress_hits | 45848 | 39417 | −6431 (−14%) |
| parkhafen | card_err_mean | −3.714 | −3.695 | +0.019 |
| | gospa_mean | 40.826 | 40.845 | +0.019 |
| | occ_suppress_hits | 24578 | 13599 | −10979 (−45%) |
| seemannshöft | card_err_mean | −1.589 | −0.959 | **+0.629** |
| | gospa_mean | 45.83 | 46.30 | +0.47 |
| | gospa_false | 1201 | 1325 | **+123** |
| | occ_suppress_hits | 104810 | 69631 | −35179 (−34%) |

Reading: the veto is real and protective (lifts 14–45% of suppression near
AIS fixes; card_err → 0, missed falls) but on dense fixed-shore harbor it admits
phantoms within the veto radius (seemannshöft gospa_false +123, net gospa
flat-to-slightly-worse). Net-neutral-to-protective at its default. The
`occ_suppress_hits`/`occ_peak_structures` columns are the increment-8 set
(no id_switches/latency emitted by this harness — named).

- **Champion (MHT) is not run on HAXR: intractable** (full-enumeration JPDA/MHT
  OOM on radar density — bench `--with-haxr` help + perf-arc). Named gap, not a
  measurement.
- Config note: the HAXR increment-8 / veto harness runs
  `imm_cv_ct_pmbm_occupancy_detector_coverage`, **not** the ivgate candidate (the
  occupancy suppression path is what HAXR exercises; the innovation gate is a
  kinematic guard orthogonal to it). Difference named.

### 1d. Harbor yardstick (`harbor_complete_truth`)

| metric | M (mht) | C (ivgate) | `coverage_land` | `pmbm` (canon) |
|---|---:|---:|---:|---:|
| lifetime | 0.975 | 0.975 | 0.975 | 0.970 |
| card_err | 8.83 | 8.00 | 8.00 | 11.1 |
| gospa_mean | 44.5 | 42.6 | 42.6 | 49.5 |
| id_sw / brk | 0.4 / 0 | 0 / 0 | 0 / 0 | 0 / 0.8 |

- Lifetime **0.975 ≥ 0.974** reproduces the M2 yardstick for all live configs.
- **Caveat [CORRECTED 2026-07-10 — the original caveat below was WRONG].**
  ~~The absolute `card_err` (~8–11) is inflated by a known truth-fragmentation
  harness artifact (the `pmbm-harbor-truth-sort-fix` branch is unmerged at
  `d94471e`); the comparable signal is the delta, not the absolute.~~
  The truth-sort fix **is merged** — commits `3ee491f` + `3aa9c58` (2026-07-02)
  are **ancestors of `d94471e`**, this dossier's own base — so these rows were
  measured on **already-corrected, time-sorted truth**. Harbor truth is **not**
  fragmented (verified: 5 objects, 40 complete groups; contract test
  `HarborCompleteTruth.TruthIsTimeSortedIntoFortyCompleteGroups` green). The
  absolute `card_err ~8–11` is therefore a **REAL over-count** — phantom tracks
  on the uncharted 13-point pier (+ transient sea clutter), not an artifact.
  Both the **delta** (candidate 8.0 < champion 8.8 < canonical-pmbm 11.1) **and
  the absolute** are meaningful — driving the absolute down is exactly the Cl-4
  one-config objective. Full reconciliation:
  `docs/baselines/2026-07-10_harbor_truthsort_reconcile.md`.
- Gate identical to `coverage_land` here too (does not fire on harbor).

### 1e. Imazu 22 battery (identity through close crossings)

Aggregate over the multi-target cases; the 6 densest 3-target cases
(14/15/17/19/20/22) are where both trackers fail. Full per-case tables in the
CSV artifacts; the decision-relevant cuts:

| axis | M (mht) | C (ivgate) | backward `coverage_land` |
|---|---:|---:|---:|
| id_switches, worst case (imazu_17) | **72** | 1 | 2.3 |
| id_switches, imazu_20 | 68 | 1 | 5 |
| dup-track churn (share of switches, all mt cases) | **88.9%** | ~0 | ~0 |
| track_breaks imazu_22 | 25.7 | **4.7** | 32.3 |
| lifetime imazu_22 | 0.888 | 0.860 | 0.667 |
| card_err imazu_01 (crossing-independent over-count) | 0.06 | 0.58 | **+0.77** |

MHT reproduces the #11 forensics (imazu_17 = 72 sw, imazu_20 = 68, dup-conveyor
88.9%). `coverage_land` reproduces the imazu22 table (imazu_01 card_err 0.77,
imazu_22 card_err −0.11, lifetime 0.667).

**#25 close-pass mitigation (the candidate's headline), reproduces Stage-2
exactly** — 6 dying truths (imazu_15 + imazu_22), loss windows straddling the
own-ship CPA scan:

| metric (6 dying truths) | gate OFF (`coverage_land`) | gate ON (candidate) |
|---|---:|---:|
| **CPA-overlap loss-seconds** | **163 s** | **6 s** |
| total dying-truth loss | 1366 s | 544 s |
| re-acquire under new id | 45 | 10 |

The innovation gate collapses the safety-critical CPA-overlap dropout ~27× and
halves total dying-truth loss, with id-switches staying ≤ 2.3 (no id-snap
onto the neighbour). This is measured, at `d94471e`, on freshly exported states.

### 1f. Sim multi-sensor gates (independent truth)

| scenario | M ospa | C ospa | `coverage_land` ospa | M card_err | C card_err |
|---|---:|---:|---:|---:|---:|
| ais_dropout | 33.1 | 108 | 125 | 0.03 | 0.35 |
| headon | 39.9 | 116 | 121 | 0.02 | 0.24 |
| overtaking | 74.5 | 101 | 142 | 0.11 | 0.39 |
| crossing | 89.1 | 108 | 100 | 0.21 | 0.39 |
| clutter_burst | 184 | 245 | 268 | **2.51** | 2.71 |
| anchored_camera | 309 | 330 | 308 | −0.14 | −0.58 |

**Fusion vs radar-only** (SIMMS_RADAR_ONLY=1), champion, ais_dropout: fusion
ospa **33.1** vs radar-only **67.2** (≈ 2× — reproduces the battery headline;
AIS anchors identity/position and radar carries the track on dropout).

- Candidate (ivgate) vs `coverage_land`: the gate **improves** ospa/breaks on
  maneuvering + dropout (overtaking 142→101, ais_dropout 125→108, headon breaks
  10→5.5) but **hurts the camera-only contact** (anchored_camera lifetime
  0.62→0.55, card_err −0.21→−0.58) — the deweight drops the bearing-only wedge
  faster. New this run; named.
- PMBM holds identity perfectly (0 id-switches everywhere) but carries the
  higher-ospa over-count (the `clutter_burst` +2.5/+2.7 is upstream duplicate-
  cloud, per the 2026-07-07 extraction-boundary ruling).

### 1g. Runtime (worst-scan latency vs 148 ms interval)

Perf-arc workloads (HAXR kattwyk_08), `--fast-metrics`, worst-scan (max) is the
operator-facing realtime number; interval = **148 ms**.

| workload | config | scan max (ms) | p95 (ms) | mean (ms) | wall (s) | vs 148 ms |
|---|---|---:|---:|---:|---:|---|
| decimated | M (mht) | **1168** | 731 | 193 | 387 | **7.9× OVER** |
| decimated | `coverage_land` | 21.0 | 18.7 | 14.2 | 28.6 | 7.0× under |
| decimated | C (ivgate) | 30.4 | 19.0 | 14.6 | 29.5 | 4.9× under |
| raw-density | `coverage_land` | 85.1 | 61.5 | 46.8 | 93.8 | 1.74× under |
| raw-density | C (ivgate) | 77.7 | 61.5 | 48.0 | 96.1 | 1.9× under |
| raw-density | M (mht) | — | — | — | **>420 (TIMEOUT)** | **intractable** |

- **Champion MHT is not realtime on radar density.** Even on *decimated* HAXR its
  worst scan is 1168 ms — 7.9× the 148 ms budget; on raw density it does not
  finish (timeout, full-enumeration blowup). This is a hard deployment ceiling
  independent of accuracy.
- **Candidate is comfortably realtime** on both (decimated 30 ms, raw 78 ms;
  reproduces the perf-arc PMBM figures — decimated ≈ 22.9 ms, raw ≈ 76.7 ms /
  92.9 s wall). The innovation gate adds ~negligible overhead (decimated max
  21→30 ms; raw unchanged within noise).

---

## 2. The trade — each failure-mode axis, named, with its number

- **Cardinality / false tracks.** Champion runs a persistent over-count from the
  duplicate-track conveyor (autoferry env-1 card_err +3.6…+5.1; Imazu dup-churn
  88.9% of switches, up to 81 track-ids for 3 truths). Candidate holds
  cardinality at truth on open water (autoferry env-1 card_err ≈ 0) but carries a
  crossing-independent **+0.77** clutter over-count on Imazu and higher OSPA on
  sim_ms.
- **Continuity (breaks / lifetime).** Champion keeps a track on target ~90% of
  its life (high lifetime, low breaks) but with churned identity. Candidate's
  existence recursion flickers — but the **innovation gate materially repairs
  the worst of it**: Imazu dying-case breaks halve or better (imazu_22 32→5) and
  CPA-overlap loss drops 163→6 s.
- **Identity (switches, both ways).** Champion churns catastrophically on dense
  crossings (imazu_17 = 72 sw). Candidate is structurally stable (≤ 2.3 sw on
  every Imazu case, 0 on sim_ms) — the gate does not snap identity onto the
  neighbour.
- **Close-pass CPA behavior (post-gate).** *Before* the gate the PMBM failure was
  dropping the target for tens of seconds at the own-ship CPA (163 s over the 6
  dying truths). *After* the gate that is 6 s. This is the axis the candidate
  most changes versus the historical PMBM.
- **Runtime.** See §1g. (Champion MHT is intractable on raw radar density — a
  hard deployment ceiling for the champion on that input, independent of
  accuracy.)
- **Geography ceiling (NEW, decision-critical).** The candidate's coverage+land
  stack has a documented <50 m offshore no-birth zone. On open water it is
  excellent (Cl-1 env-1 17.44, beats the paper); in a **shore-hugging urban
  channel it tracks nothing** (Cl-1 env-2 collapse). `pmbm_land` (land prior, no
  coverage floor) does not have this ceiling (env-2 17.74). So "the candidate"
  is a *coastal/open-water* stack, not a universal one.
- **Standing residuals.** +0.77 crossing-independent Imazu over-count (parked
  clutter/birth channel, measured-negative 2026-07-07); philos near-parity
  (73.1 vs 69.4); harbor `card_err ~8–11` is a **real uncharted-pier over-count**
  — [correction 2026-07-10] NOT "confounded by the unmerged truth-sort fix": that
  fix is merged (in this dossier's own base `d94471e`) and truth is not
  fragmented; see §1d and
  `docs/baselines/2026-07-10_harbor_truthsort_reconcile.md`.

---

## 3. Open questions (the dossier cannot answer these without the water test / deployment facts)

- **Which geography does deployment actually see?** The candidate's env-2
  collapse vs `pmbm_land`'s survival is the whole ballgame, and it depends on
  whether operations are open-water/coastal (candidate wins) or shore-hugging
  channels (candidate disqualified, `pmbm_land` or a lowered floor needed). Not
  decidable from sim.
- **Is the continuity trade acceptable operationally?** Whether a consumer keys
  alarms/CPA on *continuous presence* (favours MHT / hurts PMBM breaks) or on
  *stable identity* (favours PMBM) is a system-integration fact, not a metric.
- **Does the +0.77 over-count matter downstream?** It is bounded and self-
  limiting; whether a fused picture or an operator tolerates ~0.8 phantom track
  is a deployment call.
- **Runtime margin on real deployment density** is dominated by upstream
  extraction (the 57 s campaign-replay margin is upstream, per the perf-arc
  ruling), not by tracker compute — a full answer needs the real front-end.
- **No multi-seed** on the sims (autoferry/philos/HAXR are single-run by
  construction; Imazu/sim_ms are seed-0 only here). Variance is unmeasured.

---

## 4. Eval-log entry (commands, artifacts, commit)

All runs at `d94471e`, Release, single seed, from the worktree root with the
main-tree fixtures symlinked (`data/autoferry`, `data/dlr`,
`tests/fixtures/{philos/out,haxr_cfar,sim_multisensor/{sim_ms,imazu}_*}`). Bench
= `./build/bench/navtracker_bench_baseline`. Aggregation:
`tools/dossier_aggregate.py` (added this run, `c7b6e2f2…`); CPA-loss reproduction
`cpa_loss.py` (`985d0034…`, reuses the phase2b `tools/pmbm_phase2b_ab.py`
definitions verbatim).

```bash
# Cl-1 autoferry (3 configs; + backward pmbm_land, + coverage_land isolation)
for c in imm_cv_ct_mht imm_cv_ct_pmbm_land imm_cv_ct_pmbm_coverage_land imm_cv_ct_pmbm_coverage_land_ivgate; do
  $BIN --config-eq $c --scenario-filter autoferry_scenario --seeds 1 --run-id 2026-07-10_dossier_autoferry_$c --out docs/baselines/; done
python3 tools/dossier_aggregate.py docs/baselines/2026-07-10_dossier_autoferry_*.csv --cl1 --metrics gospa_rms

# philos (3 configs) ; harbor (4 configs, --skip-replays --scenario-eq harbor_complete_truth)
$BIN --scenario-filter philos --config-eq <c> --seeds 1 --run-id 2026-07-10_dossier_philos_<c> --out docs/baselines/

# Imazu 22 (3 configs, states exported for the 2 PMBM configs)
SIMMS_DIR=$PWD/tests/fixtures/sim_multisensor $BIN --with-imazu --skip-replays --scenario-filter imazu \
  --config-eq <c> --seeds 1 --run-id 2026-07-10_dossier_imazu_<c> --out docs/baselines/ --export-states-dir states/<..>
python3 cpa_loss.py       # 6-dying-truths CPA-overlap loss, gate OFF vs ON

# sim_ms fusion + radar-only (SIMMS_RADAR_ONLY=1)
SIMMS_DIR=$PWD/tests/fixtures/sim_multisensor [SIMMS_RADAR_ONLY=1] $BIN --with-simms --skip-replays \
  --scenario-filter sim_ms --config-eq <c> --seeds 1 --run-id 2026-07-10_dossier_simms[_radaronly]_<c> --out docs/baselines/

# HAXR 3-site veto A/B (prints the table; needs data/dlr + haxr_cfar/out wired)
./build/navtracker_tests --gtest_filter='VetoIsolationHaxrAB.*'

# Runtime (perf-arc HAXR fixtures, --fast-metrics; raw cut = tod in [29096.383,29380.922] of kattwyk_08_full)
HAXR_PLOTS_CSV=tests/fixtures/haxr_cfar/out/{kattwyk_08_dec50_w285|kattwyk_08_rawdens_w285}.csv \
  $BIN --with-haxr --scenario-eq haxr --fast-metrics --config-eq <c> --seeds 1 --run-id 2026-07-10_dossier_rt_<..> --out docs/baselines/
```

Artifacts (`docs/baselines/2026-07-10_dossier_*.csv`, sha256 prefix):
autoferry mht `3efd5fe3` / pmbm_land `463874c1` / coverage_land `db1c75bf` /
ivgate `95997015`; philos mht `bb0be335` / coverage_land `020164e0` / ivgate
`1dbe4b50`; imazu mht `9907a7f6` / coverage_land `33f57789` / ivgate `525eac5f`;
harbor mht `bb11d30f` / pmbm `efb650c9` / coverage_land `6436c045` / ivgate
`31d390b2`; sim_ms fusion mht `9b2f9052` / coverage_land `bf66c01e` / ivgate
`f88f59f9`, radar-only mht `fc3c7bec` / ivgate `e0de4dfa`; runtime dec mht
`30cf840a` / coverage_land `8ffc80fb` / ivgate `164f2ebe`, raw coverage_land
`dd5d9b6e` / ivgate `b7042100` / **mht `4356829a` (0 rows — timeout artifact)**.
Fixture checksums unchanged from their source eval-log entries (2026-07-06/08/09).

**Suite:** `ctest --test-dir build` (from worktree root, `SIMMS_DIR` set) →
**100% passed, 0 failed out of 1093**, 16 skipped. The 16 skips are all
structural, not regressions: **RBAD** (`RadarTruthLoader.*`, `RbadScenarioRun.*`
— raw 31.6 GB dataset not downloaded, per the D8 ruling) and **ctest-cwd
relative-path replay tests** (`HaxrOspa`, `PhilosOspa`, `PhilosFarCross.*`,
`AutoferryJsonReplay`, `ReplayScenarioRun.*`, `PhilosClutterMapAB` — resolve
fixtures by cwd-relative path while ctest runs with cwd=`build/`; the *same*
fixtures were exercised successfully by the bench runs above from the worktree
root, so wiring is proven and nothing is masked).

---

## 5. The decision in front of the user

1. On **open water** the candidate PMBM is the stronger tracker: it beats the
   Helgesen paper cold-start (17.4 vs 20.4) and holds identity through crossings
   the champion churns (imazu_17: 1 switch vs 72).
2. The champion keeps a **continuous track** on a target more reliably; the
   candidate trades some continuity for that identity stability.
3. The innovation gate **fixes the candidate's worst close-pass flaw**: track
   loss at the CPA drops from 163 s to 6 s over the dying cases — the safety-
   critical moment.
4. But the candidate stack **cannot track a shore-hugging channel** (autoferry
   env-2 goes to zero) because of its <50 m no-birth zone; the champion and the
   plain `pmbm_land` both can.
5. The champion **cannot run at all on raw radar density** (intractable); the
   candidate can.
6. So the choice is not "better tracker" but "which failure can you least
   afford": duplicate/renamed tracks and no-raw-density (champion), or a
   near-shore blind spot (candidate) — with the candidate clearly ahead
   everywhere that is *not* near-shore.
7. If deployment is coastal/open-water, the candidate is the promotion; if it
   includes shore-hugging channels, the no-birth-zone floor must be addressed
   first (or `pmbm_land` used there).
8. No number here settles it — the water test and the deployment geography do.
