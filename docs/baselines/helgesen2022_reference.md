# Helgesen et al. 2022 — paper reference + comparison

Helgesen, Vasstein, Brekke, Stahl, "Heterogeneous multi-sensor tracking
for an autonomous surface vehicle in a littoral environment",
*Ocean Engineering* 252 (2022) 111168.
[ScienceDirect (paywalled)](https://www.sciencedirect.com/science/article/pii/S0029801822005753).
Local PDF: `docs/references/S0029801822005753-helgesen-2022-heterogeneous-multisensor-littoral.pdf`.

Dataset: <https://github.com/Autoferry/sensor_fusion_dataset>.

## Paper methodology (extracted from §5.8 and §7)

- **GOSPA** with `c = 20 m`, `p = α = 2`, reported **as RMS** across
  time steps (paper §5.8 lines 935-938, Tables 6/7 captions). The 20 m
  cutoff is chosen to match the track-truth assignment threshold used
  elsewhere in the paper.
- Tracker: **asynchronous multi-sensor VIMM-JIPDA**. Their full-fusion
  headline configuration is `L,R,IR,EO` = lidar + radar + EO cameras +
  IR cameras (the same physical sensors navtracker's autoferry replay
  exposes).
- Aggregation: results reported **per environment**, not per scenario.
  Env 1 = scenarios 2-6 (open water). Env 2 = scenarios 13/16/17/22
  (urban channel).
- posRMSE reported per target (Havfruen ; Gunnerus in env 1;
  Havfruen ; Jetboat in env 2).
- "Break.L" is break *length* in seconds (cumulative duration of
  track-loss intervals across the env). We do not have a directly
  comparable metric — `track_breaks` is *count* of break intervals
  per truth.

navtracker side: bench run
`docs/baselines/gospa20m_20260613T174620Z.csv`, single seed, canonical
config `imm_cv_ct_mht` (IMM CV+CT inside TOMHT with IPDA+VIMM
lifecycle and per-sensor detection tables). `MetricsParams` now
defaults to `gospa_cutoff_m = 20.0` to match the paper. `gospa_rms` is
the new aggregate matching the paper's "GOSPA reported as RMS".

## Per-scenario navtracker numbers (`imm_cv_ct_mht`, c = 20 m, p = α = 2)

| Sc | env | GOSPA mean | GOSPA RMS | GOSPA p95 | pos_rmse | breaks (count) | lifetime |
|---|---|---:|---:|---:|---:|---:|---:|
| 2  | 1 | 37.5 | 40.9 | 63.4 | 8.6  | 1.5 | 0.958 |
| 3  | 1 | 45.5 | 46.4 | 58.6 | 25.7 | 1.5 | 0.872 |
| 4  | 1 | 40.6 | 42.8 | 60.4 | 11.4 | 0.5 | 0.937 |
| 5  | 1 | 41.2 | 42.4 | 61.7 | 19.4 | 1.5 | 0.913 |
| 6  | 1 | 41.7 | 44.4 | 63.3 | 34.2 | 3.0 | 0.908 |
| 13 | 2 | 24.2 | 24.6 | 31.8 | 9.9  | 1.0 | 0.773 |
| 16 | 2 | 27.8 | 28.4 | 35.5 | 10.8 | 1.5 | 0.851 |
| 17 | 2 | 31.0 | 31.3 | 37.5 | 36.3 | 2.5 | 0.902 |
| 22 | 2 | 46.4 | 47.0 | 58.3 | 32.2 | 3.5 | 0.837 |

For reference (not in paper): **philos** GOSPA RMS = 76.3, pos_rmse =
38.4 m, lifetime = 0.295 — the low lifetime is honest (most philos
vessels report AIS only twice ~10 s apart in this 20-s fixture).

## Per-environment aggregate (RMS-of-per-scenario-RMS proxy)

| Env | Scenarios | navtracker GOSPA RMS | navtracker pos_rmse (mean per scenario) | navtracker breaks (count per scenario) |
|---|---|---:|---:|---:|
| 1 (open water)   | sc2-6        | **43.4** | 19.8 | 1.6 |
| 2 (urban channel)| sc13/16/17/22| **33.9** | 22.3 | 2.1 |

**Caveat.** A perfectly apples-to-apples aggregate would pool every
per-step GOSPA across an environment's scenarios and then RMS those.
The bench currently emits only per-scenario summaries. RMS-of-per-
scenario-RMS is equivalent only when scenarios share equal step
counts, which they do not. Treat the aggregate column as a proxy.
T-GOSPA (PMBM plan phase 4) would give a principled cross-scenario
trajectory metric.

## Comparison vs Helgesen 2022 (L,R,IR,EO full fusion)

| Env | Paper GOSPA RMS | navtracker GOSPA RMS | Δ |
|---|---:|---:|---:|
| 1 (open water)   | 20.37 | 43.4 | **+23 m (≈ 2.1×)** |
| 2 (urban channel)| 30.97 | 33.9 | +3 m (≈ 1.1×)      |

| Env | Paper posRMSE Havf./other | navtracker posRMSE (per-sc mean) |
|---|---|---:|
| 1 | 38.91 / 9.43  (Havfruen / Gunnerus) | 19.8 |
| 2 | 83.53 / 50.49 (Havfruen / Jetboat)  | 22.3 |

| Env | Paper Break.L (s) | navtracker breaks (count/sc) |
|---|---:|---:|
| 1 | 86.3  | 1.6 |
| 2 | 200.2 | 2.1 |

| Env | Paper ANEES | navtracker nees_mean (per scenario, range) |
|---|---:|---|
| 1 | 15.84 | 4-80 (sc2-6 range, sc5 ≈ 80; see eval log 2026-06-13) |
| 2 | 51.90 | 20-1000+ (sc22 NEES ≈ 1000, sc17 ≈ 470) |

### Read

- **Env 2 (urban channel): navtracker is essentially on par** with
  the paper on GOSPA (33.9 vs 31.0). pos_rmse is much better — but
  partly because the paper averages Havfruen + Jetboat and Jetboat
  was particularly hard for them (50.49 m alone) due to track
  divergence onto docked-boat EO returns (paper Fig. 25). navtracker
  doesn't yet replay individual targets separately to confirm a
  fully apples-to-apples per-target comparison.
- **Env 1 (open water): navtracker is ~2× worse on GOSPA** (43.4 vs
  20.4). pos_rmse is ostensibly better (19.8 vs 38.91/9.43 mean
  24.2) — but the paper's high Havfruen RMSE (38.91 m) comes
  specifically from "track coalescence on Gunnerus" in scenarios 2
  and 5 (paper Fig. 24) where the IR cameras lock Havfruen's track
  onto Gunnerus. navtracker doesn't exhibit that failure mode but
  pays a much larger GOSPA penalty for *cardinality* errors — track
  breaks (1.6 count/scenario for us, but at 87 s of cumulative break
  length for them across the environment we don't have a directly
  comparable scalar to know which is worse in total).
- **Filter consistency is worse on both envs.** Paper env 1 ANEES =
  15.84, env 2 = 51.90 — both well above the χ²-floor of 2 for a
  consistent filter (paper also runs hot, but less so than us). Our
  sc5 NEES = 80, sc22 ≈ 1000. The eval-log 2026-06-13 entries
  document this as backlog item 12 (filter overconfidence on real
  bearing-dominated data). Both trackers underestimate uncertainty;
  ours more aggressively.

### Where the env 1 gap likely sits

GOSPA at `c = 20 m` charges `c²/α = 200` per missed truth per step.
With our 1.5-2 breaks per scenario and breaks lasting some seconds,
the cardinality penalty dominates. The paper's headline VIMM-JIPDA
configuration sits at GOSPA 20 — which means *over the same evaluation
clock* their cardinality penalty is much smaller, either because (a)
their tracks recover faster from misses, (b) their breaks happen
during low-cardinality stretches that contribute less, or (c) their
break-length integration captures something we count differently.
Confirming this requires per-step GOSPA breakdown — queued.

## What can move navtracker's number toward the paper

In priority order:

1. **JIPDA upgrade** (sota-roadmap §2). The paper's tracker *is*
   VIMM-JIPDA. We have IMM-MHT instead — a different but related
   class. Closing this gap is the obvious next algorithmic step.
2. **Inter-sensor registration biases** (backlog item 9). The paper
   carefully calibrates each sensor's mounting offset against
   GNSS-RTK truth; we do not, and unmodelled biases show up as
   posRMSE and inflated NEES.
3. **Item 12 (NEES calibration)** — we are over-confident on real
   bearing-dominated data; honest covariances widen gates and reduce
   spurious breaks.
4. **T-GOSPA + per-target separation** in the bench so the
   comparison can be per-target like the paper.

## Per-target separation (TODO)

The paper's posRMSE column is `(Havfruen ; second_target)`. Our bench
aggregates pos_rmse across all assigned truths and reports a single
number per scenario. Adding per-truth posRMSE to MetricsResult would
let us compare per-target. AutoFerry's `truth.id` distinguishes the
vessels so the bookkeeping is small.

---

# 2026-07-08 re-score — Cl-1 cold-start head-to-head with the current stack

**Everything above this line is the frozen 2026-06-13 record and is left
untouched.** The 2026-06-13 table was produced on `imm_cv_ct_mht` with the
*pre-UKF* EKF inner filter, before the UKF promotion (`6106ec7`), the PMBM
Phase 4–7 work, and the bench truth-sort fixes. This section re-scores the
same protocol on the current stack and adds the PMBM canonical, per the
`docs/superpowers/plans/2026-07-08-cl1-coldstart-ticket.md` ticket. It is a
MEASUREMENT-only pass: **no config or algorithm was changed.**

Protocol unchanged: GOSPA `c = 20 m`, `p = α = 2`, reported as RMS; env-1 =
sc2–6 (open water), env-2 = sc13/16/17/22 (urban channel). Per-environment
aggregate = **RMS-of-per-scenario-`gospa_rms`** (the same proxy the frozen
table uses; caveat above still applies). Single seed (autoferry replays are
single-run by construction).

Provenance:
- worktree `cl1-coldstart-rescore` @ `7f5cd17`, Release, g++/Conan per
  `reference_build_env`. Autoferry fixtures symlinked from the main tree.
- `docs/baselines/cl1_rescore_mht_20260708.csv`
  `sha256 38590dc730dc7c0df22f6afd9d4912acd47d1151de924134ad15620645be3abd`
- `docs/baselines/cl1_rescore_pmbm_20260708.csv`
  `sha256 3698967080fad5f0eb502d726e964f0d68d2f7381c73aad377db9c765a38b82f`
- `docs/baselines/cl1_drift_ekf_20260708.csv` (Phase-0 drift ablation)
  `sha256 b411972a3466e1095debb4be9869d0609f4106f97dce0dfdd6581a08f8e9a313`

Exact commands (from the worktree root, fixtures reachable via cwd):

```
./build/bench/navtracker_bench_baseline --run-id cl1_rescore_mht_20260708 \
  --config-eq imm_cv_ct_mht        --scenario-filter autoferry_scenario --seeds 1 --out docs/baselines/
./build/bench/navtracker_bench_baseline --run-id cl1_rescore_pmbm_20260708 \
  --config-eq imm_cv_ct_pmbm_land  --scenario-filter autoferry_scenario --seeds 1 --out docs/baselines/
./build/bench/navtracker_bench_baseline --run-id cl1_drift_ekf_20260708 \
  --config-eq imm_cv_ct_mht_ekf    --scenario-filter autoferry_scenario --seeds 1 --out docs/baselines/
```

## Phase 0 — reproduction of the frozen row (finding: partial)

The frozen row does **not** reproduce identically on the current stack, and
the drift decomposes cleanly (env aggregate GOSPA RMS):

| Stack | env-1 | env-2 |
|---|---:|---:|
| Frozen 2026-06-13 (`imm_cv_ct_mht`, pre-UKF EKF) | 43.4 | 33.9 |
| Current `imm_cv_ct_mht_ekf` (pre-UKF filter preserved as ablation) | **39.42** | **33.91** |
| Current `imm_cv_ct_mht` (UKF is now the default inner filter) | 34.37 | 28.57 |

- **env-2 reproduces exactly** under the pre-UKF filter (33.91 vs 33.9). Its
  entire 33.9 → 28.57 shift is the **UKF default flip** (`6106ec7`, "Cl-2 #3:
  promote UKF inside IMM to canonical inner filter", 2026-06-20).
- **env-1 does not fully reproduce even under EKF** (43.4 → 39.42, −9%). So
  ~44% of the env-1 drift is *non-UKF* — merged bench changes between the
  frozen commit `b77e67c` and HEAD; dominant candidates are the truth-sort
  fixes (`3ee491f` "sort truth in additive builders", `3aa9c58` "close
  remaining truth-sort gaps") and the GOSPA localization/missed/false
  decomposition wiring (`711cf45`). The remaining −13% (39.42 → 34.37) is the
  UKF flip. The exact per-commit split was not bisected (out of scope for a
  measurement pass).

**The 43.4 row is stale; the current honest `imm_cv_ct_mht` cold-start
baseline is 34.37 (env-1) / 28.57 (env-2).** Per the ticket, the table is not
silently rebased — the arbiter decides which number is the new baseline.

## Phase 1 — the decision-relevant table (current stack)

Per-environment aggregate GOSPA RMS, four conditions × two configs, vs the
paper's full-fusion (L,R,IR,EO) headline row:

| Condition | Config | env-1 | env-2 |
|---|---|---:|---:|
| no-AIS (**cold start** — the Cl-1 claim) | `imm_cv_ct_mht` (UKF) | 34.37 | 28.57 |
| no-AIS (**cold start**) | **`imm_cv_ct_pmbm_land`** | **18.62** | **17.74** |
| truth-AIS injected (calibration row) | `imm_cv_ct_mht` (UKF) | 5.18 | 5.16 |
| truth-AIS injected (calibration row) | `imm_cv_ct_pmbm_land` | 3.43 | 8.94 |
| — | **Helgesen 2022 (paper)** | **20.37** | 30.97 |

**Headline: PMBM cold-start (no-AIS) env-1 GOSPA RMS = 18.62, which beats the
paper's 20.37.** env-2 is beaten comfortably too (17.74 vs 30.97). The ×2.1
env-1 gap the frozen table reported is **closed** — the cold-start,
no-anchor claim is met by the current PMBM canonical without any tuning.
(`imm_cv_ct_mht` alone narrows to 34.37 but does not beat the paper.)

Per-scenario env-1 breakdown, no-AIS (the historically pathological
sc3/sc5/sc6 included):

| Sc | config | gospa_rms | pos_rmse | breaks | lifetime | id_sw | card_err |
|---|---|---:|---:|---:|---:|---:|---:|
| 2 | mht        | 35.15 | 10.65 | 3.0  | 0.935 | 38   | +5.06 |
| 2 | pmbm_land  | 18.42 |  8.98 | 60.0 | 0.809 | 5    | +0.39 |
| 3 | mht        | 36.67 | 28.06 | 1.5  | 0.889 | 41   | +4.69 |
| 3 | pmbm_land  | 20.83 | 15.46 | 42.0 | 0.532 | 1.5  | −0.26 |
| 4 | mht        | 33.55 | 13.79 | 1.0  | 0.879 | 16   | +4.29 |
| 4 | pmbm_land  | 14.38 |  9.53 | 14.0 | 0.785 | 5.5  | −0.13 |
| 5 | mht        | 34.02 | 21.05 | 2.5  | 0.911 | 49.5 | +3.90 |
| 5 | pmbm_land  | 20.43 | 20.16 | 33.5 | 0.691 | 10.5 | −0.21 |
| 6 | mht        | 32.26 | 28.13 | 3.0  | 0.870 | 36   | +3.59 |
| 6 | pmbm_land  | 18.35 | 13.44 | 30.5 | 0.655 | 14.5 | −0.24 |

## Mechanism (from the GOSPA decomposition — no extra runs)

The whole MHT → PMBM env-1 improvement is a **cardinality (false-track)**
effect, visible in the summed GOSPA `false` component:

- `imm_cv_ct_mht` runs a persistent **over-count**: `card_err ≈ +3.6…+5.1`,
  `gospa_false ≈ 830…1070`, `id_switches 16…49`. This is the duplicate-track
  conveyor (backlog-11 diagnosis, 2026-07-08): MHT keeps *a* track on the
  target almost continuously (low breaks, high lifetime) but sheds and
  re-spawns identities and spurious siblings, and GOSPA charges a full
  false-track penalty for each.
- `imm_cv_ct_pmbm_land` holds cardinality at truth: `card_err ≈ 0` (−0.13…
  −0.39), `gospa_false ≈ 63…160` (~7× smaller), `id_switches 1.5…14.5`.
  Localization is comparable; the missed component is slightly *higher*
  (PMBM's existence recursion flickers → breaks 14…60/scenario, lifetime
  0.53…0.81), but at `c = 20 m` a break-step costs a bounded miss while a
  duplicate costs a full false — so the trade nets strongly in PMBM's favour.

This directly confirms the standing **Cl-2 #2** hypothesis (comparison-
baselines): folding existence + association into one PMBM recursion makes the
env-1 over-count "moot", and it does — without touching a knob.

## The operational trade (framing, not a winner declaration)

The two configs fail in *opposite* ways; GOSPA at `c = 20 m` scores one of
those failure modes (per-step cardinality/localization) and is blind to the
other (continuity/presence):

- **PMBM** — best GOSPA, best pos_rmse, near-perfect cardinality, stable
  identity; **worst track continuity** (breaks 14–60/scenario, lifetime down
  to 0.53). Failure mode that hurts operationally: a target's track *drops
  and reacquires* repeatedly — bad when a consumer keys an alarm/CPA on
  continuous presence. Coincides with the close-pass track-loss elevated as
  backlog **#25**.
- **MHT (UKF)** — best continuity/presence (keeps a track on target ~90% of
  its life); **worst cardinality** (persistent +4…+5 over-count, high
  id-churn). Failure mode: the operator sees duplicate/renamed tracks for one
  vessel.

For the Cl-1 headline claim ("beat the paper in cold deployment without an
anchor") the metric the paper is scored on is GOSPA, and PMBM beats it
(18.62 < 20.37). The continuity caveat is a real deployment consideration but
is a *different* axis than the claim.

## Verdict (implementer read — arbiter decides)

Ticket verdict **(a): the env-1 gap is closed** (PMBM no-AIS env-1 = 18.62 ≲
20.4; in fact below the paper). Recommendation: the Cl-1 "beat in cold
deployment without anchor" claim card flips ✅ on the PMBM canonical, with the
continuity trade recorded. Per the ticket this is a **checkpoint** — Phase 2
(NEES-per-scenario deep dive + a sim observability control) is **not** entered
without arbiter go, and the north-star edit is left for the arbiter to land.
