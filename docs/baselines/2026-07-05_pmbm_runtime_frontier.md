# PMBM runtime probe — profile + knob sweep (2026-07-05)

**Branch:** `pmbm-runtime-probe` (worktree, off master `907bae9`).
**Scope:** MEASUREMENT ONLY. No default touched, no algorithm changed. The only
code change is an env-gated compute-knob override block in
`core/benchmark/Config.cpp::makePmbmConfig()` (UNSET env ⇒ bit-identical), used
so one binary can vary a single knob per run without recompiling — mirrors the
`HAXR_*` scenario env overrides in `ReplayScenarioRun.cpp`.

Ticket: `docs/superpowers/plans/2026-07-05-pmbm-runtime-probe-ticket.md`.

## Workload (denominator for everything)

- Fixture: `tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv`
  (md5 `304cdeb8e81f03cbddb52d629fab22a9`), 163 724 plot rows, 285 s window,
  decimated (clustering-first) 55–100 plots/scan regime.
- Config: `imm_cv_ct_pmbm_coverage_land` (the increment-8 arm).
- Bench: `navtracker_bench_baseline --with-haxr --scenario-eq haxr
  --config-eq imm_cv_ct_pmbm_coverage_land`, `HAXR_PLOTS_CSV` pointed at the
  decimated fixture.
- Build: RelWithDebInfo (`-O2 -g -DNDEBUG`) `+ -fno-omit-frame-pointer`.
- Machine: 8 cores; the tracker runs **single-threaded** (CPU 99 %).

### Baseline (steady-state, warm machine)

| quantity | value |
|---|---|
| wall (bench `wall_seconds`) | **~515 s** steady-state (cold-start first run 640 s — see machine-drift note) |
| peak RSS (`/usr/bin/time -v`) | **~83 MB** |
| processing ratio | 515 s / 285 s ≈ **1.8× slower than realtime** (decimated; the ~20× figure from increment-8 was raw density) |
| gospa_mean | 104.26 m |
| card_err_mean | **48.76 tracks** (large over-count — the known philos/haxr over-count) |
| lifetime_ratio | 0.104 |
| id_switches | 0 |

**Machine-drift note (quiet-machine caveat, per ticket).** The very first runs of
a batch clock ~640/611/602 s; once the CPU settles (thermal/frequency) the run
time locks to **515.0–516.1 s (±0.2 %)**. All frontier wall numbers below are
taken in the settled regime, with a discarded warm-up run per batch. RSS and
accuracy are unaffected by drift.

## Step 1 — profile

_Profiler:_ **gprof** (`-pg` RelWithDebInfo build), **not `perf`**. `perf record`
is blocked on this host (`perf_event_paranoid=4`; lowering it needs root, which is
not available non-interactively here). gprof needs no privileges.

**Instrument caveat (read before trusting the percentages).** gprof is a
*different instrument* from the perf the ticket specified, with real distortions
at `-O2`: (1) its `mcount` call-count instrumentation perturbs what it measures,
biasing toward high-call-count functions; (2) inlining mis-attributes inlined
callee time to the caller. So treat the numbers below as **bucket-level ranking,
not percentage-precise**. They are trustworthy *here* only because the top bucket
beats the next by ~30× — far outside gprof's distortion band. The
tracker-vs-metric split within `hungarianAssignment` is exact (it comes from
gprof's call-count attribution, not sampling). No pre-existing phase timers exist
to reconcile against (checked).

### Hotspot buckets (self-time, instrumented run; 1 998 scans)

| bucket | ~% total | top symbols |
|---|---|---|
| **Assignment solver — tracker** (`murtyKBest → hungarianAssignment`) | **85.2 %** (377.0 s / 165 721 calls) | `hungarianAssignment` |
| **Bench metric scoring** (`OSPA/GOSPA/TGOSPA → hungarianAssignment`) | **10.5 %** (46.6 s) | `hungarianAssignment` from `tgospa`, `assignPerStep`, `gospaComponents`, `ospaGreedy` |
| Cost-matrix construction (KF/IMM predicted-meas + likelihood/gate) | ~2.7 % | `ImmEstimator::gate`, Eigen `ldlt`/`partial_lu`, `predictMeasurement`, `measurementResidual` |
| Hypothesis bookkeeping (birth/merge/enumerate self) | ~1 % | `buildAdaptiveBirthCandidates`, `mergeBernoulliDuplicates`, `bhattacharyyaState` |
| Memory (alloc/copy/Eigen temporaries) | ~0.5 % | `__do_uninit_fill_n`, `Matrix` ctors/`resize`, `TrajectoryPoint` copies |
| Occupancy / static layer | **< 0.1 %** | (not in top 30 — the coverage/occupancy layer is *not* a runtime cost on this workload) |

### Is it Murty or the cost matrix? — **Neither, as framed.**

It is the **linear-assignment solver** (`hungarianAssignment`, Hungarian/LSAP),
at **85 % of runtime** from the tracker alone — beating cost-matrix construction
(~2.7 %) by ~30×, and Murty's *enumeration* is ~0 because this config runs K=1.
The root cause is a **K=1 inefficiency in `murtyKBest`** (`core/association/Murty.cpp:75-116`):
the `while` loop pops the seed assignment (satisfying K=1), but then
*unconditionally* runs the child-generation loop — one `hungarianAssignment` per
assigned row (**N ≈ 82** on this workload) — and only re-checks `size < K`
*after* the loop, discarding every child it just computed. So each scan pays
**1 + N ≈ 83** Hungarian solves where K=1 needs exactly **1**; ~98.8 % of the
solves are thrown away.

Consequences that drive Step 2 and the Step-3 call:
- The cost scales with the **child-loop trip count N** = number of *assigned*
  rows in the seed (Bernoullis/births that won a finite-cost cell). ⇒ `r_min`
  (prunes Bernoulli rows outright) is the primary knob lever; `gate_threshold`
  is secondary — it doesn't shrink the matrix Hungarian factorises, but a tighter
  gate can push marginal Bernoullis to misdetection so fewer rows are assigned,
  which *can* lower N (the sweep prices this); `max_ppp_components` bounds birth
  rows; `trajectory_window_scans` is cold (~0.04 %).
- The reported wall time carries **~10.5 % benchmark-only overhead** from the
  harness's own per-scan OSPA/GOSPA/TGOSPA assignment scoring — a deployed
  consumer would not pay this. The realtime-margin denominator should note it.
- **Step-3 target is concrete, cheap, and low-risk:** an early-exit
  `if ((int)out.assignments.size() == K) break;` (or a `k>1` guard) before the
  child loop in `murtyKBest` eliminates the wasted expansions — an ~83×
  reduction in the dominant bucket at K=1, projecting the ~515 s run toward
  ~70–90 s (assignment 377 s → ~5 s; everything else unchanged). Bit-identical
  output at K=1 (the discarded children never affect the result). *Not
  implemented here — this ticket is measurement-only.*

## Step 2 — knob sweep

Compute-relevant knobs enumerated from `PmbmTracker::Config` at the values
`imm_cv_ct_pmbm_coverage_land` actually sets:

| knob | shipped value | kept? | why |
|---|---|---|---|
| `gate_threshold` | 20.0 (χ², very loose) | **yes** | fewer gated (Bernoulli×meas) pairs ⇒ smaller cost matrix + fewer estimator updates. Verified live to move output. |
| `max_ppp_components` | 200 | yes | PPP likelihood sums + birth cost |
| `r_min` | 1e-5 | yes | higher floor ⇒ fewer Bernoullis ⇒ smaller matrices |
| `trajectory_window_scans` | 50 | yes (confirm) | pure history bookkeeping, zero algorithmic effect; RSS is tiny so expected non-lever |
| `max_global_hypotheses` | 10 | **NO — dropped** | **inert under K=1**: each parent emits exactly one Murty child, so the global-hypothesis count never exceeds 1 and the cap never binds (verified: `MAXHYP=1` byte-identical output). To make it a lever you must first raise K. |
| `k_best_per_hypothesis` | 1 | no | already the minimum; raising only *adds* cost |
| birth / PDA / merge knobs | — | no | accuracy levers, not cost drivers |
| `dedup_miss_pd` | false | no | inert under `use_sensor_activity=true` (coverage model owns the miss signal) |

### OFAT frontier (steady-state warm; Δ vs same-batch baseline 442.0 s)

Full data: `2026-07-05_pmbm_runtime_frontier.csv`. Within-batch noise floor ≈ 0.2 %.

| knob | value | wall Δ | gospa | card_err | lifetime | verdict |
|---|---|---|---|---|---|---|
| (shipped) | — | 0.0 % | 104.262 | 48.7626 | 0.1045 | baseline |
| `r_min` | **1e-2** | **−5.8 %** | 104.261 | 48.7615 | 0.1045 | best; accuracy byte-identical |
| `r_min` | 1e-3 | −3.9 % | 104.261 | 48.7615 | 0.1045 | accuracy-neutral |
| `r_min` | 1e-4 | −2.1 % | 104.262 | 48.7626 | 0.1045 | accuracy-neutral |
| `gate_threshold` | 9.0 | −0.8 % | 104.262 | 48.7626 | 0.1045 | in the noise |
| `gate_threshold` | 13.8 | −0.5 % | 104.262 | 48.7626 | 0.1045 | in the noise |
| `max_ppp_components` | 50 | −0.1 % | 104.262 | 48.7626 | 0.1045 | inert |
| `max_ppp_components` | 100 | −0.1 % | 104.262 | 48.7626 | 0.1045 | inert |
| `trajectory_window_scans` | 0 | −1.1 % | 104.262 | 48.7626 | 0.1045 | cold (RSS 83→81 MB) |

**The frontier is flat.** No knob buys a fast-dev-grade speedup. This *confirms*
Step 1: the dominant cost (Murty's K=1 child expansion, ~83 solves/scan) scales
with the per-scan **measurement + measurement-driven-birth** row count, which no
config knob controls. `r_min` nudges it by pruning Bernoulli rows (the small,
purely-Bernoulli part of N), which is why it's the only knob that moves at all;
`gate` and `max_ppp` are inside the noise; `trajectory_window` is cold as
predicted. `max_global_hypotheses` was **excluded** — inert under K=1 (each
parent emits one Murty child; the cap never binds; verified `MAXHYP=1`
byte-identical). `k_best`, birth/PDA/merge, and `dedup_miss_pd` excluded per the
table above.

### Fast-dev config (priced, honest)

**Recommendation: `PMBM_PROBE_RMIN=1e-2` (i.e. `cfg.r_min = 1e-2`).** It is the
only accuracy-safe win. Gate suite (KEEP safety) — **byte-identical to baseline
on all three**, so no regression to price:

| scenario | gospa | card_err | lifetime | id_switches | vs baseline |
|---|---|---|---|---|---|
| harbor_complete_truth | 42.5485 | 8 | 0.975 | 0 | identical |
| dense_clutter_datum | 22.7107 | 0.425 | 0.6125 | 0 | identical |
| philos | 153.539 | 107.85 | 0.0387 | 0 | identical |

(The bench emits OSPA/GOSPA/card/lifetime/id_switches, not a dedicated
`tracks_on_keep` column — that lives in the philos label-replay test. Identical
tracking metrics here are the KEEP-safety proxy; the dedicated label test should
still be the gate before any *default* change.)

**But be honest about the size:** −5.8 % (haxr) / −7.2 % (philos) is a free win,
not a fast-dev multiplier. It does **not** deliver the "run hour-scale replays
and A/Bs cheaply" goal the ticket wanted from a fast-dev config. There is **no
config-only fast-dev config** on this workload — the flat frontier is the proof.

- **Fast-dev use:** set `r_min = 1e-2` for a free ~6 %, accuracy-neutral. Marginal.
- **Candidate default change:** `r_min = 1e-2` is gate-green and accuracy-neutral
  on every scenario tested, so it is a *defensible* (if marginal) default-tighten
  candidate — **flagged for the arbiter, not applied.** Should still clear the
  dedicated philos KEEP-label test and the determinism test before promotion.

## Recommendation on Step 3 (code optimization): **strongly FOR**

The profile is the argument. 85 % of runtime is the assignment solver, and
~98.8 % of those solves are K=1 child expansions Murty computes and discards
(`Murty.cpp:75-116`). No knob can reach that cost — the flat frontier proves it.
The fix is a **~1-line early-exit** (`if ((int)out.assignments.size() == K)
break;` before the child loop): **bit-identical output at K=1**, projecting the
dominant bucket ~83× smaller and the whole ~515 s run toward **~70–90 s (~6×)**.
Low-risk, high-payoff, config-independent. Two adjacent, larger follow-ups the
profile also supports (each its own decision): (a) a sparse/gated LSAP so the
solver skips `+∞` cells instead of factorising the dense padded matrix; (b) the
bench's own per-scan OSPA/GOSPA/TGOSPA scoring is ~10.5 % of wall — worth a
`--fast-metrics`/stride option so profiling runs aren't self-taxed.

Against proceeding: none that survive the profile. The only caveat is that this
was measured with gprof, not perf (see the instrument caveat); if the arbiter
wants perf-grade confirmation before spending Step-3 effort, re-run once
`perf_event_paranoid` can be lowered — but the 85 %-vs-2.7 % gap and the exact
call-count attribution (165 721 of 186 202 solves from `murtyKBest`) make the
conclusion robust to gprof's distortion.
</content>
