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

## Probe results (2026-06-23)

Two diagnostic measurements were run on master before parking
Phase 9:

**Probe A — adaptive K with K_cap=3** (was K=5 in iter 1). Baseline
= `docs/baselines/pmbm_phase8_20260622.csv` (shipped K=1). Probe
output saved at `docs/baselines/phase9_probe_kcap3_*_20260623.csv`.

Synthetic scenarios (gospa_mean):

| scenario | delta vs K=1 |
|---|---:|
| ais_dropout | −2.23% |
| clock_skew | −9.28% |
| crossing | −6.16% |
| crossing_dropout | −2.57% |
| **dense_clutter** | **−13.12%** |
| head_on | −6.17% |
| non_cooperative | +1.20% |
| overtaking | −5.04% |
| parallel_targets | −1.94% |
| speed_change | −6.39% |

Net synthetic: 9 wins (up to −13%), 1 small regression (+1.2%) —
strong positive signal.

Autoferry scenarios (gospa_mean):

| scenario | delta vs K=1 |
|---|---:|
| sc2 | −3.32% |
| **sc2_anchored** | **+8.53%** |
| sc3 | −4.95% |
| sc3_anchored | +2.75% |
| **sc4** | **−14.90%** |
| sc4_anchored | −8.69% |
| sc5 | −0.43% |
| sc5_anchored | +5.60% |
| sc6 | −6.20% |
| sc6_anchored | +8.56% |
| sc13 | +9.47% |
| **sc13_anchored** | **+44.97%** |
| sc16 | +11.87% |
| **sc16_anchored** | **+38.56%** |
| sc17 | −4.66% |
| sc17_anchored | +8.61% |
| sc22 | −0.60% |
| sc22_anchored | +14.24% |

Net autoferry: 8 wins, 10 regressions. The anchored regressions on
sc13 / sc16 are **worse** at K=3 than they were at K=5 (+25..+33% in
iter 1), so the simple K-dilution hypothesis is refuted — going to
lower K doesn't monotonically help.

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

## Recommendation

Park Phase 9 as a multi-day refactor that does NOT have a confirmed
ROI. The hypotheses-2 and 3 in the section above need to be probed
first (each ~½-1 day) before the structural refactor's risk/reward
becomes worth taking.

If a future session wants to revisit, the order is:
1. Diagnostic dump of sc13_anchored MBM weights K=1 vs K=3 vs K=5
   (½ day) — pinpoints where the regression begins.
2. Cross-read of `PoissonMBMtarget_update.m:268-330` vs our
   `enumerateChildren` new-target-row handling (½ day) — checks for
   the new-target-log-weight double-count.
3. Decide refactor vs reroll based on what 1 + 2 surface.

This doc supersedes the parking note in
`docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md` parking
lot item #1 — the "adaptive K is structural" diagnosis is now
explicitly downgraded from "confirmed" to "candidate hypothesis,
unfalsified".
