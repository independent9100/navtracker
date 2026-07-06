# Implementer prompt — hot-path mechanical sympathy (perf round 3: targeted, verified list)

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: 2026-07-06 arbiter code-read of the round-2 hot bucket (IMM
per-mode measurement update). Unlike round 2's leftovers this ticket has a
VERIFIED target list, not a hypothesis: the wart in
`IEstimator::logLikelihood` / `ImmEstimator::logLikelihood` was confirmed by
reading the code. Round-2 writeup: `docs/baselines/2026-07-06_perf_round2.md`.

---

You are working in the navtracker repo (C++17, CMake+Conan, read `CLAUDE.md`
first). Mission: make the PMBM/IMM likelihood hot path mechanically efficient
— fewer decompositions, less recomputation, no heap churn — WITHOUT changing
what it computes. Stretch goal: raw-density workload under 57 s (≥5× realtime
margin); report honestly if it falls short. This does NOT change the standing
deployment ruling (front-end extraction stays mandatory for accuracy) — the
win is full-shift replay speed for data campaigns + margin headroom.

## Two change classes — different proof obligations

- **Class A — byte-identical** (pure hoisting of identical computations with
  preserved op order). Proof: full suite green + byte-compare of bench metric
  CSVs on BOTH workloads (excluding wall/scan_proc rows).
- **Class B — epsilon-class** (fp reordering: shared decomposition,
  solve-instead-of-inverse, fixed-size kernels). Results shift at ~1e-15;
  determinism stays (still deterministic), but frozen prints move. Proof:
  full suite (fix any print-pinned tests by the freeze-commit rule — upgrade
  to tolerance/config-independent assertions, don't just re-pin), PLUS the
  full standing gate suite priced: harbor_complete_truth, dense_clutter_datum,
  philos KEEP replays, NEES/NIS sanity. Metric deltas must be fp-noise-sized
  (≲1e-6 relative); anything visibly larger means you changed the math —
  STOP and report, don't rationalize.

Do Class A first and commit it separately, so its byte-identical proof isn't
contaminated by Class B. One commit per logical step, each with before/after
wall + per-scan p99/max (the round-2 latency columns exist — use them).

## Setup

1. Worktree: `git worktree add ../navtracker-perf3 -b hotpath-mech-sympathy`
   off current master. Own build dir; Conan sandbox gotcha per CLAUDE.md.
2. Re-measure YOUR OWN baselines first on both workloads (machine state
   drifts; round-2 numbers are the reference, not your denominator):
   - Decimated: `tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv`
     (md5 `304cdeb8e81f03cbddb52d629fab22a9`), reference ~41.6 s.
   - Raw: cut recipe in the 2026-07-06 eval-log entry (LC_ALL=C awk,
     tod∈[29096.383, 29380.922], 299 981 rows, md5
     `64aadfb462eb26b022368e371ce5ff42`), reference ~141–163 s.
   - Invocation: `navtracker_bench_baseline --with-haxr --scenario-eq haxr
     --config-eq imm_cv_ct_pmbm_coverage_land`, `HAXR_PLOTS_CSV` set.
   Quiet machine; discard one warm-up run per batch.

## Verified targets (in order)

### T1 (Class B, biggest confirmed wart) — one decomposition, not two

`core/estimation/EstimatorDefaults.cpp:38-59` and
`core/estimation/ImmEstimator.cpp` `logLikelihood` both do
`S.determinant()` AND `S.inverse()` — two LU decompositions of the same
matrix in the innermost loop (per track × measurement × mode). Replace with
ONE `Eigen::PartialPivLU` (or LDLT): `.determinant()` from the factorization
and `lu.solve(y)` for the Mahalanobis term (never form the inverse). Preserve
the `safe_det` guard semantics exactly (det > 0 && finite, else 1e-300 path).
AUDIT for the same pattern everywhere on the hot path: `gateDistance`,
estimator `update()` implementations (EKF/UKF/IMM/PF), MHT scoring — this
base-class fix benefits every estimator, so fix it at each site it occurs.

### T2 (Class A) — hoist measurement-independent prediction work

In the PMBM cost/likelihood loop, `predictMeasurement(model, x, sensor_pos)`
and `H·P·Hᵀ` are recomputed per (track, measurement) pair. For
**Position2D / PositionVelocity2D** the prediction and H do not depend on the
measurement (H is a selector; z_pred is track state) — hoist per (track,
mode) per scan and add each measurement's `R` per pair. CAUTION: for
**Bearing2D** the prediction depends on `z.sensor_position_enu` (varies per
measurement) — do NOT hoist those; dispatch by model. Keep op order identical
(compute HPHᵀ into a temp, then + R) so this stays Class A; verify by
byte-compare. Implementation must stay inside the estimator/tracker internals
— no port or public-API change.

### T3 (Class B) — fixed-size kernels for the dominant dims

The hot matrices have known small dims (S: 2×2 for Position2D/Bearing-pair,
4×4 for PositionVelocity2D; state 4 or 5). Dispatch the d==2 case to
`Eigen::Matrix2d` stack kernels (closed-form det/inverse for 2×2 is fine —
it IS the LU result up to fp order). This removes the heap allocation churn
(every call currently allocates several `MatrixXd` temporaries) and is the
cache-friendliness win — small matrices stop being heap-scattered. Scope it:
the likelihood/gate path first; only extend to update() if the profile still
shows allocation churn after.

### Explicitly OUT of scope

- constexpr/LUT hunting (assessed 2026-07-06: the path is data-dependent
  linear algebra, nothing worth tabling).
- The parked round-2 findings (coarse gate prefilter — result-affecting
  beyond epsilon; sparse LSAP — assignment no longer hot).
- Any behavior/algorithm change, any port/public-API change, any threading.

## Acceptance

1. Class A commit(s): byte-identical proof stated (suite + CSV byte-compare
   both workloads).
2. Class B commit(s): gate-suite pricing table (metric deltas listed, all
   fp-noise-sized), NEES/NIS sanity, determinism green, any print-pinned
   test upgraded per the freeze rule (named in the commit message).
3. Final numbers table: baseline vs after-each-commit, both workloads, wall +
   scan_proc p99/max. State the raw-density verdict vs the 57 s stretch goal.
4. Eval-log entry (dated) + a short results md in `docs/baselines/`. If T1's
   audit found the pattern in other estimators, list every site fixed.
5. Docs: this is internal mechanics — no learning-doc or integration-guide
   change expected. If you believe otherwise, say why in the handoff.
6. Budget ~1–2 days. Stop rule: if after T1+T2 the raw workload is not under
   ~90 s, report before starting T3 (the remaining headroom may not justify
   the blast radius).
7. Branch stays unmerged; handoff to the arbiter with the pricing table.
