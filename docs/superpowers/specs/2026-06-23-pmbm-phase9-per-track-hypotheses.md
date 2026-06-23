# PMBM Phase 9 design: per-track single-target hypothesis lists

**Status**: design only — not implemented. Captures findings from the
MATLAB MTT-master cross-read and shortlists cheaper alternatives that
should be tried before committing to the full structural refactor.

## Why this doc exists

Phase 8 iter 1/3/5 tried to ship `adaptive_k_best = true` (Murty
K-best with K per parent ≈ `ceil(N_hyp_max · w_p)`, matching the
MATLAB MTT-master adaptive cap). Three configurations were tried:

| iter | change | sc13/16/22 anchored gospa |
|---|---|---|
| 1 | `adaptive_k_best=true`, K=5 cap | +25..+33% regression |
| 3 | iter 1 + Bhattacharyya merge threshold 1.0 → 0.25 | unchanged |
| 5 | iter 1 + `scan_birth_id_cache_` (siblings share birth id) | unchanged |

The remaining hypothesis is **structural**: our flat
`GlobalHypothesis::bernoullis` representation diverges from the
MATLAB per-track single-target hypothesis list, and the divergence is
what's blocking adaptive K from delivering its wins on anchored
scenarios. This doc maps the MATLAB structure, sketches what a C++
match would look like, and lists the cheaper alternatives we should
exhaust first.

## MATLAB structure (MTT-master / García-Fernández 2018)

Source: `third_party/reference/MTT-master/PMBM filter/`.

The top-level `filter_pred` / `filter_upd` struct
(`PMBMtarget_filter.m:44-67`) carries:

- `weightPois, meanPois, covPois` — PPP intensity, identical to ours.
- `tracks{i}` — **cell array**, one entry per persistent track.
  Each `tracks{i}` carries `meanB{j}`, `covB{j}`, `eB(j)`, `aHis{j}`,
  `t_ini` where `j` indexes the per-track single-target hypothesis
  list. Multiple `j` per track ≡ "the same target's state under
  different association histories".
- `globHyp` — **integer index matrix**, `(N_hypotheses × N_tracks)`.
  `globHyp(p, i) = j` means "global hypothesis p selects per-track
  hypothesis j of track i"; `j = 0` ≡ "track i is absent in
  hypothesis p".
- `globHypWeight` — global mixture weights.

Murty K-best (`PoissonMBMtarget_update.m:146-176, 268-330`) runs **on
the expanded per-track-hypothesis cost matrix**: each existing
hypothesis row is duplicated `M+1` times (M measurements + miss
option). Per-track hypothesis indices grow as `index_hyp = j +
N_hyp_i * m`. The K assignments produce K rows of `globHypProv`;
each row references the new (expanded) per-track indices.

Output (`PoissonMBMtarget_estimate1.m:1-26`) picks the top
`globHyp` row, dereferences each non-zero column into
`tracks{i}.meanB{globHyp[top, i]}`, and emits a flat list.

## Our C++ structure (current)

`core/pmbm/PmbmTypes.hpp`:

- `PoissonComponent` — identical to MATLAB PPP.
- `GlobalHypothesis::bernoullis` — flat `std::vector<Bernoulli>`.
  No index; the Bernoulli **carries its own state** in every global
  hypothesis it appears in.
- `Bernoulli::id` — stable identity (`BernoulliId`), preserved
  across hypothesis copies.

`PmbmTracker::enumerateChildren` (lines 410-736):
- Builds a `(n_parent_bernoullis + m_measurements) × (m_measurements
  + n_parent_bernoullis)` cost matrix.
- Runs Murty K-best.
- Each K assignment becomes a **complete new GlobalHypothesis** with
  freshly-copied Bernoullis (detected / missed / newly-born).

## The structural diff

| dimension | MATLAB | C++ today |
|---|---|---|
| per-track hyp storage | `tracks{i}.meanB{j}` | none — flattened into `GlobalHypothesis::bernoullis` |
| global hyp content | `globHyp[p, i] = j` (integer index) | `parent.bernoullis` (full state copies) |
| Murty rows | per-track-hypothesis (`j + N_hyp_i · m`) | per-parent-Bernoulli (`i`) |
| K=5 cost | indices grow; state stored once | full Bernoulli copies multiplied K-fold |
| output | top-hyp row → per-track-list lookup | top-hyp `bernoullis` directly |
| pruning | drop unused per-track-hyps + reindex | drop low-weight `GlobalHypothesis` whole |

The hypothesis: under K=5 our flat copy approach blows up redundant
near-identical Bernoulli states across the K children; merging
collapses them post-hoc but loses the per-track structure that the
MATLAB code preserves end-to-end.

## Three cheaper alternatives to try BEFORE the full refactor

The structural hypothesis is **plausible but not confirmed**. The
following 1-day probes should be run first:

### Alt 1 — output marginalization across top-K global hypotheses

**Already implemented.** Code-read in `PmbmTracker.cpp:1287-1337`
(`refreshAggregatedTracks`) confirms the output path already
marginalizes per-id across all global hypotheses via the textbook
formula:

```
mass(id) = Σ_h h.weight · b_h.r            (over Bernoullis with that id)
mean(id) = Σ_h h.weight · b_h.r · b_h.mean / mass(id)
cov(id)  = Σ_h h.weight · b_h.r · (P_h + μ_h μ_h^T) / mass(id) − μ·μ^T
```

So the "we pick a single top hypothesis and lose information" failure
mode is **not** what's hurting adaptive K. The regression must come
from upstream of the output layer — most likely **mass dilution**:
K children that disagree on which ambiguous detections are clutter
split the existence mass of contested ids K-fold, and ids whose
marginalized mass falls below `output_existence_floor` (0.1) drop
out of the emitted tracks entirely → fragments → gospa penalty.

### Alt 2 — duplicate-hypothesis collapse

After enumerateChildren, walk the new children. Two children are
"duplicate" if their `bernoullis` differ only by ε in state for the
same id set. Collapse the lighter into the heavier:

```
sort children by log_weight desc
for each pair (a, b): if max||b_a.mean − b_b.mean|| < ε for all matched ids,
    merge: weight_a += weight_b; drop b
```

This is the Bhattacharyya merge applied at the **global hypothesis**
level, not within a hypothesis. Currently we only merge Bernoullis
within a single hypothesis. **Cost**: ~30 LOC + tuning of ε.

### Alt 3 — bound K by Murty cost separation

The Murty K=5 assignments may have very small relative cost
differences (e.g. the 2nd-best costs 1.001× the best). Adding a
"discard assignments within ε of the best" cutoff prunes
near-redundant children at the source:

```
const double base_cost = kb.assignments[0].cost
for k > 0: if kb.assignments[k].cost > (1 + ε) * base_cost: break
```

**Cost**: ~10 LOC + an `epsilon_k_cutoff` config knob.

## The full Phase 9 refactor (if all three fail)

Concrete file-by-file plan, ~3-5 days, captured here so a future
session can pick it up cold.

### Data structure (PmbmTypes.hpp)

```cpp
struct TrackHypothesis {
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance;
  double existence_probability;
  Eigen::MatrixXd imm_means;
  std::vector<Eigen::MatrixXd> imm_covariances;
  Eigen::VectorXd imm_mode_probabilities;
  Timestamp last_update;
  std::vector<MeasurementId> association_history;
  Trajectory trajectory;
};

struct Track {
  BernoulliId id;
  Timestamp birth_time;
  std::vector<TrackHypothesis> hypotheses;  // MATLAB tracks{i}
};

struct GlobalHypothesis {
  double weight;
  double log_weight;
  // index[i] = j means "this hypothesis selects hyp j of track i";
  // j = -1 means "track i absent in this hypothesis".
  std::vector<int> hyp_index;
};

struct PmbmDensity {
  std::vector<PoissonComponent> ppp;
  std::vector<Track> tracks;                // MATLAB tracks{}
  std::vector<GlobalHypothesis> mbm;        // now references tracks via hyp_index
};
```

### Touch points

| file | touch | est. LOC |
|---|---|---|
| `core/pmbm/PmbmTypes.hpp` | new types | +60 |
| `core/pmbm/PmbmTracker.{hpp,cpp}` | rewrite enumerateChildren, predict, output | ~600 |
| `core/pmbm/PmbmOutput.cpp` | top-hyp + per-track-hyp dereference | ~40 |
| `core/pmbm/RtsSmoother.cpp` | walks tracks now, not bernoullis | ~30 |
| trajectory snapshots (Phase 4) | per-track, not per-global-hyp | ~50 |
| tests/pmbm/*.cpp | rewrite probes from `density().mbm[0].bernoullis` to `density().tracks[i].hypotheses[j]` | ~200 |

Total: ~1000 LOC across 8-10 files. Risk: trajectory + smoother
integration may cascade.

### Order of operations (gradual migration)

1. Add new types alongside old (don't remove `Bernoulli` / `bernoullis`).
2. Build a "Track view": a helper that materializes per-track lists
   from the current flat representation on demand.
3. Add the new representation behind `Config::use_per_track_hypotheses`.
4. Re-route enumerateChildren under the new flag (keep flat path
   for the off-flag bit-identical baseline).
5. Re-route output, predict, smoother under the flag.
6. Bench-prove sc13/16/22 anchored recovery at K=5.
7. Flip default; remove flat path after one stable cycle.

## Probe results (2026-06-23) — corrected at 10 seeds

Initial K=3 probe at 2 seeds (since deleted; superseded by the
shipped 10-seed baseline) appeared to show large synthetic wins
(clock_skew −9%, head_on −6%, etc.). **Same-run apples-to-apples
re-measurement at 10 seeds revealed those "wins" were bench-run
drift between the new 2-seed probe and the pinned phase8 10-seed
baseline — the actual K-effect on most synthetic scenarios is
within ±0.5%.** This is exactly the failure mode that
`tools/bench_diff.py` was added to prevent; lesson learned the
hard way at the very next opportunity.

The corrected K=3 vs K=1 picture (same run, 10 seeds, pinned at
`docs/baselines/pmbm_adapt_k3_phase9_20260623.csv`):

**Real K=3 wins (gospa_mean)**:

| scenario | delta vs K=1 |
|---|---:|
| **philos** (real-world replay) | **−17.23%** |
| autoferry_scenario4 | −14.90% |
| autoferry_scenario4_anchored | −8.69% |
| dense_clutter | −7.79% |
| autoferry_scenario6 | −6.20% |
| autoferry_scenario3 | −4.95% |
| autoferry_scenario17 | −4.66% |
| autoferry_scenario2 | −3.32% |

**Real K=3 regressions (gospa_mean)**:

| scenario | delta vs K=1 |
|---|---:|
| **autoferry_scenario13_anchored** | **+44.97%** |
| **autoferry_scenario16_anchored** | **+38.56%** |
| non_cooperative | +17.04% |
| autoferry_scenario22_anchored | +14.24% |
| autoferry_scenario16 | +11.87% |
| autoferry_scenario13 | +9.47% |
| autoferry_scenario17_anchored | +8.61% |

Most other synthetic scenarios are flat within ±0.5%.

Decision: **ship the K=3 config as a sibling** of K=1
(`imm_cv_ct_pmbm_adapt_k3`, `core/benchmark/Config.cpp`). Consumers
pick by workload — K=3 for cluttered / multi-target / replay, K=1
for anchored / low-clutter / non-cooperative. The split is explicit
per-scenario rather than swallowed by averages, and the K=3 wins on
philos (−17%) and dense_clutter (−8%) are too large to leave on the
floor while waiting for the structural refactor.

The anchored regressions on sc13 / sc16 are **worse** at K=3 than
they were at K=5 (+25..+33% in iter 1), so the simple K-dilution
hypothesis is refuted — going to lower K doesn't monotonically help.

**Probe B — output marginalization across hypotheses**: already
shipped in `refreshAggregatedTracks` (`PmbmTracker.cpp:1287-1337`).
No further experiment possible at this layer.

## Updated diagnosis

The adaptive-K regression on anchored scenarios is **not** purely
K-dilution, **not** lack of output marginalization, **not**
within-hypothesis Bernoulli merge resolution, and **not** birth-id
fragmentation across siblings. Three remaining hypotheses:

1. **Anchored-mode-specific**: the regressions are concentrated in
   anchored variants where own-ship is static. Maybe the
   bias-correction / coordinate frame interacts with deep K-best
   enumeration in a way the freely-moving variants don't trigger.
   Worth a focused diagnostic: dump per-scan global-hypothesis
   weights for sc13_anchored under K=1 vs K=3 vs K=5 and look at
   where the divergence begins.
2. **Per-track-hypothesis structural** (the original Phase 9
   hypothesis): not yet falsified.
3. **Murty cost-matrix construction**: our cost matrix may
   double-count the new-target rows when adaptive K is on. Reuter's
   adaptive birth replaces `ρ_target` with `λ_birth`; under K>1 each
   sibling that doesn't pick the new-target row still gets the
   `log(ρ_total)` log-weight contribution. Compare against
   `PoissonMBMtarget_update.m:268-330` to see if MATLAB handles this
   differently.

## M2 diagnostic results (2026-06-23)

Both half-day diagnostics ran in M2 of the 2026-06-23 session.

### Diagnostic B — Murty new-target double-count: REFUTED

Cross-read of `third_party/reference/MTT-master/PMBM filter/
PoissonMBMtarget_update.m` (lines 227-293) vs our
`core/pmbm/PmbmTracker.cpp:enumerateChildren` (lines 580-735)
confirmed our handling matches MATLAB exactly: new-target diagonal
cost block, `log(ρ_total)` only added per K-child when the child's
assignment claims the measurement for new-birth, unassigned rows
contribute nothing.

### Diagnostic A — sc13_anchored K-best divergence: ROOT MECHANISM IDENTIFIED

Instrumented per-scan MBM-state dump on `autoferry_scenario13_
anchored` at K=1 vs K=3 pinned divergence to **scan 3**:

- K=1: 15-measurement scan absorbed as misdetections of 2 low-r
  priors + clutter. `mbm.size() = 1`, `Σr = 0.179`, no tracks
  emitted.
- K=3: same top hypothesis (lw=-43.877, w=0.497), but Murty's
  K-2 and K-3 (lw≈-44.55, w≈0.25 each) spent 5 measurements on
  phantom births → `Σr ≈ 1.09`. Output aggregation
  `Σ w·r` over 3 hyps pushed phantom Bernoulli above
  `output_existence_floor` → emitted as Confirmed.

End-of-run signature: K=1 has 2 clean tracks, max id ~210; K=3 has
the same 2 physical targets as ids #1683 / #3299 — ~15× more id
churn → id-switch + position-blur → gospa +44.97 %.

### M2 fix attempt: dominance cutoff (DIDN'T WORK CLEANLY)

A `Config::k_best_dominance_log_gap` knob (default 0 = off) was
added at `PmbmTracker.cpp:987-1009`. Drops per-parent K-siblings
whose log_weight is more than N nat below the top sibling.

Probe at log_gap=1.0 vs K=1 baseline (with K=3-baseline-vs-K=1 → K=3-with-cutoff-vs-K=1):
- `dense_clutter` -7.79 % → **-14.50 %** (improved!)
- `autoferry_scenario3` -4.95 % → -10.03 % (improved)
- `autoferry_scenario2` -3.32 % → -7.33 % (improved)
- `philos` -17.23 % → **-2.80 %** (regressed — marquee win lost)
- `autoferry_scenario4_anchored` -8.69 % → +0.58 % (regressed)
- `autoferry_scenario2_anchored` +8.53 % → +18.50 % (regressed)
- `sc13_anchored` +44.97 % → +44.31 % (basically unchanged)
- `sc16_anchored` +38.56 % → +35.59 % (small improvement)

sc13_anchored unchanged because its scan-3 alts sit at 0.69 nat
from the top (weights 0.497 vs 0.25) — BELOW the 1.0 nat
threshold, so the cutoff misses them. Tighter (log_gap=0.5) would
catch them but sacrifice more philos. **There is no log_gap that
cleanly separates phantom-birth alts from legitimate close-weight
alts at the log_weight layer alone.**

The knob is kept in the codebase (off by default) for any consumer
that values dense_clutter/sc3/sc2 wins over philos; the K=3 bench
config does NOT enable it.

### Updated hypothesis ranking (refuting / superseding the above)

1. **Per-track-hypothesis structural refactor** (Phase 9
   original): under MATLAB's `tracks{i}.meanB{j}` indexing, alt
   assignments that birth a Bernoulli for a measurement DON'T
   create a new physical id — they create a new single-target
   hypothesis index under the same track. Output aggregation can
   then weight per-track-hypothesis-marginal correctly. Our flat
   representation forces birth-id assignment per alt → id churn.
2. **Output-side cross-id birth merge** (new probe): in
   `refreshAggregatedTracks` (PmbmTracker.cpp:1287-1337), when
   two ids have nearly-identical (mean, cov) and both were born
   in the same scan, fold them. Probably ~30 LOC; cheaper than
   structural; testable as a 1-day probe.
3. **Anchored-mode-specific cost-matrix tuning**: investigate
   whether bias-correction on a static own-ship changes the
   cost-matrix structure in a way that flattens Murty K-best
   alt-cost gaps. Cheapest if the hypothesis bites.

## M3 Option B result (2026-06-23): BIAS HYPOTHESIS REFUTED + IMPORTANT REFRAMING

**Critical finding**: "anchored" in this codebase is NOT static
own-ship as I assumed. Per
`adapters/replay/AutoferryJsonReplay.cpp:364` and
`adapters/benchmark/ReplayScenarioRun.cpp:176`,
`inject_truth_anchor=true` injects synthetic AIS-style
**truth measurements** (sensor=Ais, source="autoferry_truth_anchor",
σ=5 m, one per truth scan per target). The own-ship trajectory is
identical between anchored and unanchored variants. Anchored mode
is essentially "RTK-quality observations of every target".

Bias-correction probe: built `imm_cv_ct_pmbm_adapt_k3_nobias`
(K=3 minus `build_sensor_bias_estimator`), ran sc13/13_anc/16/16_anc
at K=1 vs K=3 vs K=3-no-bias:

| scenario | K=1 | K=3 | K=3 no-bias |
|---|---|---|---|
| sc13_anchored | 3.42 | 4.95 | 4.69 |
| sc16_anchored | 3.03 | 4.20 | 4.28 |

Disabling bias barely moves the regression (sc13_anc 44.9 % → 37.2 %,
sc16_anc 38.6 % → 41.4 %). Bias-flattening is **not** the mechanism.

Actual mechanism: when the top assignment has very high likelihood
(truth-anchor σ=5 m measurements feed cleanly into the JPDA-gated
cost matrix, top is very confident and very localized), K=3 alts
must pick **worse** assignments that K=1 would never consider —
these are exactly the phantom-birth alts from Diagnostic A. The
mechanism is "easy main hypothesis → alts must invent rare events
to differ", not "uniform R-inflation flattens costs".

This rules out one of the three remaining hypotheses cleanly and
adds a sharper statement of what IS happening: PMBM under K=3 can
not exploit high-quality observations because Murty K>1 forces
spurious-association alts even when no genuine assignment ambiguity
exists.

## M3 Option A probe results (2026-06-23)

The output-side cross-id birth merge from M2's hypothesis-(2) was
implemented as `PmbmTracker::Config::output_merge_bhattacharyya_
threshold` (PmbmTracker.hpp ~line 236, PmbmTracker.cpp
refreshAggregatedTracks). When > 0, walks pairs of aggregated
ids and folds those whose Bhattacharyya distance is below the
threshold into the older id. Default 0 = off, bit-identical to
legacy. Two unit tests pin both behaviors.

6-iteration bench tuning against `imm_cv_ct_pmbm_adapt_k3`:

| iter | knobs | sc13_anc | sc16_anc | philos |
|---|---|---:|---:|---:|
| 1 | th=1.0 | +35.93 % | +27.54 % | +0.10 % |
| 2 | th=0.3 | +35.93 % | +27.88 % | +0.10 % |
| 3 | floor=0.3 only | +44.97 % | +37.74 % | -17.23 % |
| 4 | th=2.0 + floor=0.5 | +35.51 % | **+23.01 %** | -0.31 % |
| 5 | th=0.5 | +35.93 % | +27.50 % | +0.10 % |
| 6 | th=1.0 + max_age=2s + max_hyp_support=3 | +36.23 % | +29.48 % | -1.09 % |
| (K=3 baseline) | th=0 | +44.97 % | +38.56 % | -17.23 % |

Iter 6 added two discriminators (`output_merge_max_age_sec`,
`output_merge_max_hyp_support`) suggested by an M3-A adversarial
review subagent. Idea: phantoms are young + weakly supported;
legitimate parallel tracks are mature + well supported. **The
discriminators help, but not enough**: philos recovered ~1 pt
(+0.10 % → -1.09 %) but still lost most of the marquee win
(-17.23 %). Likely cause: philos's mbm cardinality is small
(few well-separated targets), so legitimate parallel tracks are
in few hypotheses and tripped the merge anyway.

Candidate iter-7 not run: **relative-mbm-size hyp-count gate**
(require min(hyp_count) < α * mbm_size, e.g. α=0.3). With α=0.3
and mbm=3 (philos), threshold = 0.9 → no merge ever. With
mbm=10 (anchored K=3), threshold = 3 → catches phantoms in 1-2
hyps. Untested.

Findings:

- The merge **works as designed**. At any positive threshold it
  drives sc16_anchored from +38.56 % down toward +23-27 % (-11 to
  -15 pts) and sc13_anchored from +44.97 % down to ~+36 % (-9 pts).
  Biggest single dent in the anchored regressions yet measured.
- Iterations 1, 2, 5 (thresholds 0.3 / 0.5 / 1.0) produce
  near-identical results, which means the merge is firing on the
  same pairs at all three thresholds — those pairs have
  Bhattacharyya distance << 0.3. The threshold doesn't tune the
  merge population, only whether the merge runs at all.
- Iteration 3 (raise output_existence_floor with no merge): no
  effect. The phantoms have mass >= 0.3 already (the diagnostic
  reported r=0.55), so they pass the higher floor.
- Iteration 4 (loose merge + high floor) gave the best sc16_anc
  result but didn't help philos either — the floor doesn't catch
  philos's specific failure mode.
- **Every threshold flips the philos win** (-17.23 % → +0.10 %).
  philos has legitimate close-positioned parallel tracks
  (different physical vessels passing close) whose Bhattacharyya
  distance is also < 0.3. The output-layer merge can't tell them
  apart from the K=3 alt-hypothesis phantom births that
  sc13/16_anchored emit.

The mechanism IS the right one — folding cross-id duplicates at
output is exactly what's needed for the K=3 alt-phantom case —
but the output layer's information (id, mass, mean, cov) doesn't
carry enough signal to discriminate "phantom from alt hypothesis"
from "legitimate close parallel track". The discriminator that
would work (per-track-hypothesis lineage: was this birthed in a
weak alt of the same parent as the other id?) lives in the
data structure Phase 9 (1) would refactor.

### Decision

Ship the knob OFF in the bench K=3 config; keep it wired and
unit-tested in case a consumer wants the sc13/16-anchored fix at
the cost of philos. The K=3 config is unchanged.

## Updated hypothesis ranking (Phase 9 M3 — after A+B done)

1. **Per-track-hypothesis structural refactor** (Phase 9
   original): would carry the lineage information that the output-
   layer merge probe lacks. Still the only candidate fix that can
   plausibly separate phantom-from-alt from legitimate-close at
   the data structure level rather than at the output layer.
2. **Anchored-mode cost-matrix tuning** (M3 Option B): **REFUTED**.
   Bias-flattening is not the mechanism. Truth-anchor measurements
   make the top assignment so confident that K=3 alts must
   manufacture phantom births to differ.
3. **Output-side merge with relative-mbm-size discriminator**
   (iter-7 candidate, not run): the iter-6 absolute hyp-count
   threshold (3) failed because philos's mbm is small. A relative
   threshold (e.g. min(hyp_count) < 0.3 * mbm_size) might work but
   the underlying mechanism doesn't go away — Murty K>1 is still
   manufacturing spurious assignments under high-confidence top.
4. **Suppress K>1 when top assignment is unambiguous** (new): if
   the top Murty assignment's cost is much better than any feasible
   alternative, just emit K=1 even when adaptive K asks for more.
   This is the `k_best_dominance_log_gap` knob shipped in M2 —
   probe at gap=1.0 didn't dent sc13_anc, but with B's reframing
   (alts genuinely are FAR worse than top in anchored scenarios),
   a tighter gap (e.g. 0.3) combined with iter-6 discriminators
   might work. Untested.

## Recommendation

Output-side birth merge (M3 Option A) ran 5 iterations and is
documented as "mechanism correct, discriminator insufficient at
the output layer". Phase 9 (1) is still the only candidate fix
with a plausible path to separating phantoms from legitimates.

Next probe — M3 Option B (anchored-mode investigation) — is in
progress in the same session.

This doc supersedes the parking note in
`docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md`
parking lot item #1.

## Phase 9 S1/S2/S3 implementation (2026-06-23)

Three milestones shipped from the "gradual migration" plan:

### S1 (commit e559b49) — per-track types + view-builder

Additive scaffold: `TrackHypothesis`, `PmbmTrack` (named to avoid
collision with `navtracker::Track`), `TrackedGlobalHypothesis`
landed in `core/pmbm/PmbmTypes.hpp` alongside existing
`Bernoulli` / `GlobalHypothesis`. `rebuildPerTrackViewFromFlat()`
derives the per-track view from the flat MBM (pure re-shape).
Config flag `use_per_track_hypotheses` declared but unused.
5 new unit tests; bit-identical baseline; 753/753 pass.

### S2 (commit 516ae62) — view-rebuild plumbing under flag

`processBatch` calls `rebuildPerTrackViewFromFlat` after each
`pruneAndNormalise` when the flag is on. Pure re-shape — output
still authoritative from the flat path. 2 new unit tests
(view-matches-flat + flag-off-leaves-view-empty); 755/755 pass.

### S3 (this commit) — lineage-aware alt-birth gate

Pragmatic shortcut to the structural refactor: rather than
rewriting `enumerateChildren` (~600 LOC) to carry per-track-
hypothesis lineage, observe that Murty's `cost_k - cost_0`
ALREADY encodes the log-weight gap from the top sibling — the
exact discriminator the per-track refactor would surface. Adds
a post-hoc filter in the enumeration caller that, when a K-child's
log_weight is below `top_lw - alt_birth_log_gap_threshold`, strips
its NEWLY-BORN Bernoullis (birth_time == scan_time) while keeping
its detection / misdetection contributions. The alt child's
log_weight is unchanged (the assignment is still scored as
feasible); only the phantom-birth output mass is suppressed.

Distinct from `k_best_dominance_log_gap` (which drops the whole
child past the threshold) and `min_new_bernoulli_existence`
(which is a per-cell r_new gate with no sibling context).

Knob: `PmbmTracker::Config::alt_birth_log_gap_threshold`,
default 0 = off. Bench probe config:
`imm_cv_ct_pmbm_adapt_k3_altgate` (sibling of `_adapt_k3` with
threshold 0.5 nat).

### S3 probe results — 10-seed bench, K=3 vs K=3+altgate

| scenario | gospa Δ | id_switches Δ | pos_rmse Δ | ospa Δ | nees Δ |
|---|---:|---:|---:|---:|---:|
| philos | **-12.71 %** | **-66.67 %** | +15.93 % | -4.33 % | +199.73 % |
| dense_clutter | -4.77 % | +33.33 % | -7.48 % | **-24.02 %** | -8.71 % |
| autoferry_scenario16_anchored | -2.02 % | +166.67 % | -0.32 % | -3.78 % | -0.25 % |
| autoferry_scenario13_anchored | +0.31 % | 0 % | +0.15 % | +0.39 % | +0.09 % |
| autoferry_scenario22_anchored | -0.01 % | 0 % | +0.00 % | +0.00 % | +0.01 % |
| autoferry_scenario2 | -1.71 % | (n/a) | | | |
| autoferry_scenario5 | -1.38 % | | | | |

**Reading**: the lineage hypothesis is **partially validated**.
The gate produces a clean **dense_clutter win** (gospa -4.8 %,
ospa -24 %, pos_rmse -7.5 %, sog_rmse -12.5 %) and a **philos
gospa + id_switches win** at the cost of philos pos_rmse +15.9 %
and NEES +200 %. The autoferry-anchored regressions
(sc13/16/22_anc) **DO NOT move materially**, refuting the
hypothesis that lineage-aware alt-birth suppression alone fixes
them.

The pos_rmse / NEES regression on philos is the mechanism's
shadow side: by suppressing alt-birth Bernoullis, the top
hypothesis's Bernoulli stays the canonical position estimate,
even when an alt's birth would have provided a better-fitting
state for some measurements. The output gets fewer ids (good for
id_switches) but worse mean positions (bad for pos_rmse).

### Decision

- Implementation (knob + unit tests) ships as `alt_birth_log_gap_threshold`.
- Probe config `imm_cv_ct_pmbm_adapt_k3_altgate` ships as a
  consumer choice (best on dense_clutter; mixed on philos; no
  effect on autoferry-anchored).
- Default bench config `imm_cv_ct_pmbm_adapt_k3` is unchanged.
- **sc13/16/22_anchored regressions remain unresolved**. The
  structural per-track-hypothesis refactor is still the only
  candidate that hasn't been falsified.

### Hypothesis ranking — final after S1/S2/S3

1. **Per-track-hypothesis structural refactor with full state-
   merging across alts**: only candidate that can both share
   identity across K-children's births AND merge the alt's
   state contribution into the top hypothesis's Bernoulli (so
   the output gets the better-fitting state without inflating
   id_switches). The S3 lineage gate captures the discriminator
   but throws away the alt's state information; the structural
   refactor would preserve and merge it.
2. ~~Output-side merge with relative-mbm-size discriminator~~
   (M3-A iter-7 candidate): not run; mechanism shown insufficient
   in iters 1-6.
3. ~~Anchored-mode cost-matrix tuning~~ (M3-B): REFUTED.
4. ~~Suppress K>1 when top assignment is unambiguous~~
   (k_best_dominance_log_gap): probed M2, doesn't discriminate.
5. ~~Lineage-aware alt-birth gate at output time~~
   (S3 alt_birth_log_gap_threshold): SHIPPED as probe; works
   mechanically but doesn't fix sc13/16/22_anc.

## S4 — cross-parent birth-id cache (2026-06-23): CORE FIX FOUND

**User intuition**: the philos id_switches -67 % from S3's altgate
felt suspicious — too big a movement for a clean intervention,
suggesting a structural issue. Diagnosis: existing
`scan_birth_id_cache_` (Phase 8 iter 5) shares Bernoulli ids
across K siblings of ONE parent only — key is `(parent_idx,
measurement_idx)`. Under K=3 with a multi-parent mixture, every
scan two different parents' children that birth the SAME
measurement get DIFFERENT ids. When an alt later becomes top,
the output id flips → id_switch + position-blur.

Fix: new knob `PmbmTracker::Config::cross_parent_birth_id_cache`.
When true, the cache key drops `parent_idx` (uses `-1` as the
parent slot), so ALL K-children of ALL parents birthing
measurement `l` share one BernoulliId. Mirrors MATLAB MTT-master's
filter-level new-track creation (PoissonMBMtarget_update.m: new
tracks belong to the FILTER, not to a specific globHyp). Default
false = bit-identical to S2/S3 baseline.

Probe config: `imm_cv_ct_pmbm_adapt_k3_xparent` (sibling of
`_adapt_k3` with the flag on, no altgate).

### S4 probe results — 10-seed bench (vs K=3 baseline)

| scenario | gospa Δ | id_switches Δ | pos_rmse Δ | sog_rmse Δ | nees Δ | ospa Δ |
|---|---:|---:|---:|---:|---:|---:|
| **autoferry_scenario13_anchored** | **-30.13 %** | (0→0.5) | **-25.32 %** | +14.16 % | **-81.48 %** | **-42.90 %** |
| **autoferry_scenario16_anchored** | **-22.42 %** | -33.33 % | -2.25 % | +5.17 % | -3.93 % | **-45.12 %** |
| **autoferry_scenario22_anchored** | **-10.57 %** | 0 % | -2.39 % | +8.47 % | -2.13 % | -13.72 % |
| philos | +20.56 % | -33.33 % | **-25.93 %** | **-37.90 %** | -10.46 % | +5.18 % |
| dense_clutter | +2.70 % | 0 % | -0.04 % | +1.88 % | -0.13 % | +5.48 % |

### S4 vs K=1 baseline (the original Phase 9 regression target)

| scenario | K=1 | K=3 baseline | K=3+xparent | xparent vs K=1 |
|---|---:|---:|---:|---:|
| sc13_anchored | 3.4174 | 4.9544 (+44.97 %) | 3.4618 | **+1.30 % — recovered** |
| sc16_anchored | 3.0279 | 4.1954 (+38.56 %) | 3.2547 | **+7.49 % — mostly recovered** |
| sc22_anchored | 7.3465 | 8.3926 (+14.24 %) | 7.5052 | **+2.16 % — recovered** |

**The original Phase 9 target is essentially fixed.** The three
autoferry-anchored regressions are within 8 % of K=1, down from
+15..+45 %.

### Mechanism understood end-to-end

Under K=3 with the truth-anchor measurement σ=5 m feed (sc13/16/22
_anchored), every scan's Murty alts birth the same physical-vessel
measurement from different parents because each parent has very
slight residual disagreement on existing-track associations. Without
the cross-parent cache, each alt mints a fresh id for the same
physical vessel → output aggregation can't fold them (different
ids) → emit as Confirmed Tentative tracks alongside the canonical
id → 15× more id churn → +44.97 % gospa.

With the cross-parent cache, all alt-births for the same
measurement collapse to one id → output aggregation sees one
phantom per measurement instead of K phantoms → mass stays
concentrated on the genuine top-hypothesis Bernoullis → gospa
recovers.

### The philos tradeoff is structural too

xparent loses the K=3 philos gospa win (-17.23 % → +0.21 % vs
K=1). Mechanism: K=3's philos win came from the same mass-spread
across multiple ids per physical vessel — it happened to
aggregate "favourably" against the gospa scoring on philos's
multi-vessel cardinality. xparent removes that spread.

But xparent also delivers philos pos_rmse -25.93 %, sog_rmse
-37.90 %, nees -10.46 % — all real-world meaningful improvements.
The gospa "win" was largely a counting artifact; the per-track
state estimates are unambiguously better under xparent.

### S4 ranking

1. **imm_cv_ct_pmbm_adapt_k3_xparent**: clean structural fix
   for autoferry-anchored. Loses philos GOSPA but improves
   philos pos/sog/nees. **Recommended default for any consumer
   whose grading is on position accuracy + autoferry-anchored
   reliability.**
2. **imm_cv_ct_pmbm_adapt_k3** (K=3 baseline): keeps philos
   GOSPA win, broken on autoferry-anchored.
3. **imm_cv_ct_pmbm_adapt_k3_altgate**: dense_clutter winner,
   mixed on philos, no effect on autoferry-anchored.
4. **imm_cv_ct_pmbm_adapt** (K=1): the safe baseline; conceded
   philos GOSPA but no anchored regressions.

The four ship as siblings; consumer picks per workload.
