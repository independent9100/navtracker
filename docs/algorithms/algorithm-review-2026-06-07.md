# Algorithm Review — 2026-06-07

In-depth review of the recursive-estimation, association, and tracking stack
against textbook formulations (Bar-Shalom 2001; Julier-Uhlmann 2004; Blom &
Bar-Shalom 1988; Blackman & Popoli 1999) and against three reference
implementations: Stone Soup (vendored at `data/stonesoup/`), FilterPy's
`IMMEstimator`, and MATLAB's `trackingIMM`. Follows the project documentation
standard: Math / Assumptions / Rationale / Ways to improve.

Pre-review snapshot tagged at `pre-algo-review-fixes`.

## Production estimator inventory

| Config (`benchmark/Config.cpp`) | Filter | Motion | Association |
|---|---|---|---|
| `ekf_cv_gnn` | EKF | CV4 | GNN |
| `ekf_cv_jpda` | EKF | CV4 | JPDA (classical-enumeration) |
| `ukf_cv_gnn` | UKF | CV4 | GNN |
| `ukf_ct_gnn` | UKF | CT(5-state, ω from state) | GNN |
| `imm_cv_ct_jpda` | IMM-2 | CV5State + CoordinatedTurn | JPDA |

**Noisy-CV is not used anywhere yet** (only `ConstantVelocity5State` with two
ω-PSD values for a CT mode in `test_multi_seed_sweep.cpp`, which is not the
same thing). See "Planned: noisy-CV third mode" below — we want it.

## What is correct (and which reference it matches)

- **Sigma points.** Scaled UKF formulation: `λ = α²(n+κ) − n`,
  `L = chol((n+λ)P)`, `Wm[0]=λ/(n+λ)`, `Wc[0]=λ/(n+λ)+(1−α²+β)`,
  `Wm[i]=Wc[i]=1/(2(n+λ))`. Matches Stone Soup `functions.gauss2sigma` and
  MATLAB's `trackingUKF` (default `α=1e−3`, `β=2`, `κ=0`).
- **CV CWNA process noise.** Per-axis block
  `q·[[dt³/3, dt²/2],[dt²/2, dt]]`. Matches Stone Soup
  `ConstantVelocity` and Bar-Shalom-Li §6.2.
- **CT closed-form F.** Off-diagonal blocks `sin(ωdt)/ω`, `(1−cos(ωdt))/ω`
  with `cos(ωdt) / ±sin(ωdt)` rotation of velocity. Matches Stone Soup
  `models.transition.nonlinear.ConstantTurn.function` (after state-ordering
  permutation) and MATLAB `constturn`. CV-limit Taylor fallback below
  `|ω|<1e-6` is correct.
- **Measurement Jacobians.** Range/bearing partials
  `∂r/∂x = dx/r`, `∂β/∂x = −dy/r²`, etc. Angle-wrap residual for
  RangeBearing/Bearing models. Sensor pose offset folded into `dx, dy`.
- **IMM mixing (Blom & Bar-Shalom 1988).**
  `c_j = Σᵢ πᵢⱼ μᵢ`, `μ_{i|j} = πᵢⱼ μᵢ / c_j`,
  `x̂ⱼ⁰ = Σᵢ μ_{i|j} x̂ᵢ`,
  `Pⱼ⁰ = Σᵢ μ_{i|j} (Pᵢ + (x̂ᵢ − x̂ⱼ⁰)(·)ᵀ)`,
  mode-prob update via log-sum-exp. Matches FilterPy
  `IMMEstimator.predict`/`update` and MATLAB `trackingIMM.predict`/`correct`.
- **Heterogeneous state handling (approach 1, padding).** CV5State
  carries ω as a passive random-walk state so it lives in the same
  5-vector space as CoordinatedTurn. **FilterPy enforces this** —
  its `IMMEstimator.__init__` raises if any sub-filter has a different
  `x.shape` ("All filters must have the same state dimension"), and
  `predict()` computes `x = Σ wj · kf.x`, `P += wj · (yyᵀ + kf.P)`
  with `y = kf.x − x`, identical to our mixing. **MATLAB `trackingIMM`
  is more flexible**: it supports different state dimensions across
  sub-filters via a user-supplied `ModelConversionFcn` (default
  `@switchimm`) that maps state and covariance between model spaces —
  that is "approach 2" from the project planning notes. We chose
  approach 1 (FilterPy style) for simplicity; revisit if a CA mode
  is ever added (Mazor et al. would prefer approach 2 there).
- **JPDA / PDAF.** Gating by Mahalanobis, full enumeration of feasible
  joint events, log weights `log P_D + log N(z|ẑ,S) − log λ_C` for assigned
  measurements and `log(1−P_D)` for undetected tracks. Marginal `β_{j,t}`,
  `β_0`. Matches Bar-Shalom-Li §6.3 and Stone Soup `JPDA`/`JPDAWithEHM`
  (we lack their EHM solver for scaling, but the marginalization is the
  same).
- **PDAF soft update.** `P_post = β₀·P + (1−β₀)·(I−KH)P + K·Y·Kᵀ` with
  residual spread `Y = Σ β_m yy' − ȳ ȳ'`. Canonical PDAF covariance
  (Bar-Shalom-Li §6.4).
- **MHT branch scoring.** Per-hit `log P_D + log N(z|ẑ,S) − log λ_C`;
  per-miss `log(1−P_D)`. Standard log-likelihood-ratio score
  (Blackman & Popoli §16). N-scan trunk pruning ascends N levels and keeps
  the winning ancestor subtree.
- **Particle filter.** SIR with systematic resampling (Kitagawa 1996),
  ESS-gated trigger at `ess_threshold_`. Per-particle weight
  `∝ exp(−½ yᵀR⁻¹y)`; `1/√((2π)^d|R|)` is dropped (`R` shared, cancels
  in renormalization).

## Issues found

### 1. UKF is not actually UKF for CT — CORRECTNESS BUG

`UkfEstimator::predict` propagates each sigma point through a single
`F = motion_->transitionMatrix(dt)` computed once at the mode's ω. For
`CoordinatedTurn`, F depends on ω; sigma points that perturb ω need their
own F. Applying one F to all sigma points reduces UKF to linearized
prediction — i.e., EKF — and the entire reason for using UKF on the
CT mode (capturing the ω nonlinearity) is lost.

Stone Soup's `UnscentedKalmanPredictor.predict`
(`predictor/kalman.py:340`) passes the nonlinear `transition_model.function`
to `unscented_transform`, so each sigma point is propagated individually.
FilterPy's `UnscentedKalmanFilter.predict` uses the user-supplied `fx(x, dt)`
per sigma point. MATLAB's `trackingUKF.predict` likewise propagates each
sigma point through the user `StateTransitionFcn` ("samples representative
points around the state estimate and transforms them through the actual
nonlinear function"). Our defaults `α=1e-3, β=2, κ=0` agree with both.

**Fix.** Add `propagate(x, dt) → x_next` to `IMotionModel` (default
implementation: `F(dt) * x`). `CoordinatedTurn` and `PrescribedTurn`
override to use the closed-form per-state-ω propagation.
`UkfEstimator::predict` calls `propagate` per sigma point. CV and
CV5State keep the default. Drop `CoordinatedTurn::setOmega` (see #7).

### 2. PDAF likelihood normalization (V·P_D) not plumbed

`ImmEstimator::softUpdate` computes `Λ_j ∝ β₀ + Σ_m β_m N(y; 0, S_j)`,
which is the unnormalized proxy. The textbook form has
`Λ_j = β₀ + (1−β₀)/(V·P_D) Σ_m β_m N(y; 0, S_j)` where `V` is the gate
volume and `P_D` is the detection probability. The proxy preserves
relative ordering across modes (so mode-switching still works), but
absolute mode likelihoods used downstream (e.g. by MHT score) are off.

**Fix.** Extend `softUpdate` to accept `(p_d, clutter_density)` and use
them in the per-mode Λⱼ. JpdaAssociator already knows both. This needs
either an interface change to `IEstimator` (cleanest) or a stored
context on the Track (more invasive). We choose the interface change.

### 3. EKF covariance update uses simple form, not Joseph form

`EkfEstimator::update`: `P = (I − KH) P`. Symmetric and PD in exact
arithmetic, but loses symmetry / positive-definiteness over long tracks.
Stone Soup `ExtendedKalmanUpdater` uses Joseph form by default
(`updater/kalman.py`): `P = (I−KH) P (I−KH)ᵀ + KRKᵀ`. MATLAB's
`trackingEKF` likewise. FilterPy provides both via the `update_steadystate`
toggle and recommends Joseph for ill-conditioned cases.

**Fix.** Joseph form in both `EkfEstimator` and `ImmEstimator` update +
softUpdate paths. Marginal CPU cost; substantial numerical robustness gain.

### 4. UKF cross-cov subtraction (LOOKS BAD, IS FINE)

`UkfEstimator::update` computes `xd = sigma_pt[i] − track.state`. This
is correct because update() recomputes sigma points from the *posterior
of predict* (i.e., from the current `track.state` and `track.covariance`),
so `track.state` is the mean those new sigma points are drawn around.
Stone Soup does the same in `unscented_transform`.

**Fix.** Documentation only — add a comment so this doesn't trip the
next reader.

### 5. JPDA soft update single-H assumption is implicit

`EkfEstimator::softUpdate` and `ImmEstimator::softUpdate` linearize H, S, K
at the predicted state using `gated_measurements[0]`. This is the
textbook PDAF simplification (H depends on the state, not the
measurement value, so it's shared across gated measurements that share
the same `MeasurementModel` and sensor pose). The IMM code already
notes this in its docstring. Nothing asserts the homogeneity.

**Fix.** Assert `model` and `sensor_position_enu` are identical across
all gated measurements. Loud failure beats silent miscompute when an
adapter starts mixing sensors into one JPDA call.

### 6. MhtTracker textbook gaps — CLOSED (Hungarian K=1; Murty K>1 deferred)

What `MhtTracker::processBatch` did before:

- Per-tree: branch, k-local prune, N-scan prune (correct).
- New track if a measurement gates to no existing best-leaf.
- Emits one Confirmed track per tree from its own best leaf.

What it does now (post-fix):

- **6a. M-of-N confirmation.** `TrackTreeNode` carries `is_hit`
  (true for measurement-derived nodes, false for missed-detection
  branches). `TrackTree::countHitsInWindow(leaf, window)` walks back
  the ancestry chain counting hits. The emitted Track's status is
  `Confirmed` iff `countHitsInWindow(chosen_leaf, confirm_hits_window)
  ≥ confirm_hits_needed`. Default 2-of-3. Blackman §III.A.
- **6b. K=1 global hypothesis via Hungarian.** `TrackTreeNode` carries
  `scan_meas_idx`, the index of the scan measurement the node consumed
  (or `kNoMeasurement` for misses). At report time `solveGlobalHypothesis`
  builds a T × (M+T) cost matrix (rows = trees, cols = scan measurements
  ∪ tree-specific miss slots), enters `-best_leaf_score` for each
  feasible (tree, action) cell, forbids cross-tree miss-slots, and
  solves the LSAP. The Hungarian solver (`core/association/Hungarian.cpp`)
  is a rectangular Jonker-Volgenant variant with a BIG_M
  ∞-replacement so degenerate all-infeasible rows can't hang or UB.
  Each tree's reported state comes from its globally-selected leaf —
  no measurement is consumed by more than one tree at report time.
  Murty K=>1 deferred: when the solver had to pick a forbidden cell
  (no feasible assignment), the caller falls back to the tree's local
  best leaf so we never silently drop a track.
- **6c. Branch merging by Bhattacharyya.** `TrackTree::mergeBranches`
  iterates leaf pairs in score-descending order; for each pair with
  `B(p, q) < threshold` on the 2-D position block of the Gaussian,
  the lower-scoring leaf is dropped. Default threshold 1.0 (Mahalanobis-
  like; 0 = disabled). Runs between `pruneKLocal` and `pruneNScan` so
  trunk merging sees a deduped leaf set.

New tests:

- `tests/association/test_hungarian.cpp` — 6 unit tests covering
  square / rectangular / forbidden / degenerate / empty inputs.
- `tests/tracking/test_track_tree.cpp` — `CountHitsInWindow*`,
  `MergeBranches*`.

Remaining gap: **K>1 (Murty) deferred.** True deferred-commitment MHT
maintains the K best global hypotheses across scans and only collapses
them at N-scan trunk merge time. K=1 already enforces the cross-tree
detection-uniqueness constraint, which was the main correctness
deficiency; K>1 is a quality-of-tracking win for ambiguous scenes that
we can add later without restructuring the data flow.

**Update 2026-06-08: Murty K-best solver landed.** The K-best primitive
is implemented at `core/association/Murty.{hpp,cpp}` (8 unit tests
including all-permutations exhaustive check) and wired into
`MhtTracker::solveGlobalHypothesis` with `k_best=3` default. See
[`docs/superpowers/specs/2026-06-08-murty-k-best-design.md`](../superpowers/specs/2026-06-08-murty-k-best-design.md)
and [its impl plan](../superpowers/plans/2026-06-08-murty-k-best-impl.md).

The minimal-first-cut behaviour is: report comes from the best
(K=1) assignment, identical to before; the K-1 alternative
assignments are collected per tree (`top_k_leaves`) but NOT yet
used to protect alternative-hypothesis leaves from pruning.
`2026-06-08_murty-k3` benchmark vs `2026-06-07_with-mht`
is bit-identical on every cooperative scenario, as expected.

The *behavioural* win of K>1 requires the second step: have
`pruneKLocal` (or a new pass between scans) honour the
`top_k_leaves` set so alternative-hypothesis branches survive into
N-scan trunk merge. Spec §3 calls this the "minimal first-cut" lever;
it's a small change once we add a `protected` flag to
`TrackTreeNode`. The bigger structural upgrade — a full global-
hypothesis tree (Reid 1979 / Kurien 1990 layer 2) — remains the
long-term direction.

**Update 2026-06-08 (protected leaves): mechanism wired.** Added
`TrackTreeNode::is_protected` (one-scan-deep). After each
`solveGlobalHypothesis` call, every leaf in `assign.top_k_leaves[t]`
and its ancestor chain back to the root is flagged. `pruneKLocal`
skips protected leaves when demoting, `mergeBranches` refuses to
merge a protected leaf away, and `pruneNScan` keeps protected nodes
in addition to the winner chain (the load-bearing piece — without it
the trunk merge would kill alternative branches before they affected
the next scan). The score-delete sweep also defers a tree's drop
when any leaf is protected. Three new unit tests cover the three
pruning paths; all pass. Spec/impl plan referenced above; implementation
plan at `~/.claude/plans/jaunty-wobbling-pascal.md`.

Benchmark `2026-06-08_protected-leaves` vs `2026-06-08_murty-k3`
(both freshly regenerated at the same HEAD modulo this change):
**every MHT config is bit-identical on every available scenario**
(`0 ·`). This is consistent with the mechanism being correct: in
the current bench set (cooperative crossings/overtakings/head-on +
non_cooperative single-target + speed_change + ais_dropout + clock_skew
+ parallel_targets), Murty's K=2/3 alternatives mostly coincide with
the K=1 best — there's no genuine assignment ambiguity for protection
to defer. The behavioural win needs scenarios with dense clutter or
crossing+dropout-style ambiguity, not yet in the synthetic harness;
adding them is a follow-up. The non-MHT `ukf_ct_gnn` row shows
between-run variability that is *pre-existing* and unrelated to
protection (same drift appears between two consecutive fresh runs
without the change).

The Murty primitive is also the explicit Phase 0 dependency of the
PMBM port; landing it here means PMBM Phase 1 can call into the same
code path.

### 7. `CoordinatedTurn::setOmega` mutates a `const`-shared model

`CoordinatedTurn` exposes `setOmega(...)` via `mutable double omega_`
so `ImmEstimator::predict` can set ω per mode before reading F.
`std::shared_ptr<CoordinatedTurn>` shared across tracks would alias
this state; today the IMM sets-then-reads within a single thread so
the race window is zero, but parallel per-track predict (planned)
would race.

**Fix.** Bundled with #1: once `propagate(x, dt)` reads ω from `x`,
`setOmega` is unused and can be removed.

## Decisions

### Constant-acceleration (CA) IMM mode — DEFERRED

Ships rarely sustain accelerations that exceed radar position noise at
typical scan rates, and CT with free ω already covers heading-change
maneuvers. Adding CA to a padded-state IMM widens the phantom-dimension
gap and biases mixing during model transitions. The standard maritime
recipe (Mazor et al. 1998 survey; FFI papers; Stone-Soup maritime
examples) is CV + CT + noisy-CV — *not* CA.

We will revisit only if logged-data residual statistics show systematic
CV+CT under-fit on speed-change segments.

### Noisy-CV (high-noise CV) third mode — PLANNED, NEXT WAVE

Same state vector as CV5State, same closed-form F, but with `accel_psd`
inflated 10× – 100× to absorb unmodelled motion (CV+CT can't explain).
Cheap to add (no new state dim), no phantom-dimension growth in mixing,
covers the niche CA would have covered without its drawbacks.

Open: tune `accel_psd` from data, choose its IMM transition probabilities
(default: same as CT → makes it the "escape hatch" mode).

This is tracked as a follow-up improvement and should not be forgotten —
we want it implemented soon after the current fix wave lands.

## Ways to improve / test next

(In addition to the items in `estimation.md`, `association.md`,
`pipeline.md`, and `sota-roadmap.md`.)

1. **Noisy-CV third mode in production IMM.** Configure
   `imm_cv_ct_noisy_jpda` in `core/benchmark/Config.cpp`, sweep
   `accel_psd_noisy` on the bus-maneuvering scenario, compare against
   `imm_cv_ct_jpda` baseline. (Planned next.)
2. **UKF/CT regression test.** Once #1 is fixed, add a test that
   propagates a CT track through a 90° turn, compares the predicted
   end-point against numerical integration of the CT ODE, and asserts
   UKF predict error << EKF predict error. Today no test catches the bug.
3. **PDAF mode-likelihood normalization** (Fix #2) is the gate to
   feeding IMM-blended likelihoods into MHT scoring honestly.
4. **MHT global hypothesis** (Fix #6) unlocks honest multi-target reports.

## References

- Bar-Shalom, Li, Kirubarajan (2001), *Estimation with Applications to
  Tracking and Navigation*, ch. 5–6.
- Blom & Bar-Shalom (1988), *The IMM algorithm for systems with
  Markovian switching coefficients*, IEEE TAC 33(8).
- Blackman (2004), *Multiple Hypothesis Tracking for Multiple Target
  Tracking*, IEEE AES Magazine 19(1).
- Blackman & Popoli (1999), *Design and Analysis of Modern Tracking
  Systems*, ch. 16.
- Julier & Uhlmann (2004), *Unscented Filtering and Nonlinear
  Estimation*, Proc. IEEE 92(3).
- Mazor et al. (1998), *Interacting multiple model methods in target
  tracking: a survey*, IEEE AES 34(1).
- Stone Soup (vendored): `data/stonesoup/Stone-Soup/stonesoup/`.
- FilterPy: `filterpy.kalman.IMMEstimator`,
  `filterpy.kalman.UnscentedKalmanFilter` (Labbe 2014).
- MATLAB Tracking Toolbox: `trackingIMM`, `trackingUKF`, `trackingEKF`,
  `trackerTOMHT`.
