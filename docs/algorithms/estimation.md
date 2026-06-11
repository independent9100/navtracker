# Estimation Algorithms

Baseline state estimation for navtracker. Follows the project documentation
standard: Math / Assumptions / Rationale / Ways to improve. Cross-reference:
design spec section 11.

## 1. Constant-velocity motion model (`ConstantVelocity2D`)

**Math.** State `x = [px, py, vx, vy]ᵀ` in ENU (m, m/s).
`F(dt) = I₄` with `F(0,2)=F(1,3)=dt`, so `[px+vx·dt, py+vy·dt, vx, vy]`.
Process noise (continuous white-noise acceleration, scalar PSD `q`, per axis):
per-axis block `q·[[dt³/3, dt²/2],[dt²/2, dt]]`, placed at the (pos,pos),
(pos,vel),(vel,pos),(vel,vel) entries for each of x and y.

**Assumptions.** Near-constant velocity between updates; maneuvers absorbed by
`q`; x/y independent; 2D surface motion.

**Rationale.** Standard, robust baseline; keeps all nonlinearity in the
measurement models. Chosen over constant-acceleration (overfits) and
coordinated-turn (premature) for the first cut.

**Ways to improve / test next.** IMM mixing CV + coordinated-turn for
maneuvering vessels; tune/learn `q` per vessel class; extend to 3D if needed.
**Planned next: noisy-CV third mode for the production IMM** — same CV5State
state vector, accel PSD inflated 10×–100× to absorb unmodelled motion that
CV+CT can't explain. This is the standard Mazor-1998 maritime recipe and
the principled alternative to a CA mode (see
[algorithm-review-2026-06-07.md](algorithm-review-2026-06-07.md)).

## 2. Measurement models (`MeasurementModels`)

**Math.**
- Position2D: `h(x)=[px,py]`, `H=[[1,0,0,0],[0,1,0,0]]`.
- PositionVelocity2D: `h(x)=[px,py,vx,vy]`, `H=I₄`.
- RangeBearing2D: `r=√(px²+py²)`, `β=atan2(py,px)`;
  `H=[[px/r, py/r, 0, 0], [−py/r², px/r², 0, 0]]`.
- Residual: `y=z−h(x)`; for bearing, `y` wrapped to (−π, π].

**Assumptions.** Range/bearing relative to the ENU frame origin (own-ship
offset is applied later in normalization); `r>0` (guarded at 1e-6); Gaussian,
zero-mean measurement noise with covariance `R` provided per measurement.

**Rationale.** EKF Jacobian linearization is adequate for the mild range/bearing
nonlinearity at operational geometry and is far cheaper than UKF/particle.

**Sensor pose (added 2026-06-01).** `RangeBearing2D` and `Bearing2D` accept an
optional `sensor_position_enu` parameter (default `(0, 0)`). When supplied,
`r` and `β` are computed as `‖p_target − p_sensor‖` and
`atan2(py − sy, px − sx)`; the Jacobian's position partials are evaluated
at the shifted position. `Measurement.sensor_position_enu` carries the
sensor's ENU location at measurement time, which the adapter sets either
once (stationary shore radar) or per-measurement from `OwnShipProvider`
(mounted on a moving platform). This is what makes moving-sensor
bearing-only fusion tractable: the geometry that creates parallax is
captured in `h(x)` directly, not baked into the adapter's projection layer.
All call sites (EKF/UKF/PF/IMM update, Gating, JPDA, MHT) forward
`z.sensor_position_enu` automatically.

**Ways to improve / test next.** UKF for stronger nonlinearity or bearing-only
geometry; particle filter for multimodal cases (bearing-only before range
converges); per-sensor `R` calibration from data; sensor orientation
(boresight, pan/tilt) for body-frame bearings.

## 3. Extended Kalman Filter (`EkfEstimator`)

**Math.**
- Predict: `dt = to − last_update`; `x ← F(dt)x`; `P ← F P Fᵀ + Q(dt)`.
- Update: `(ẑ,H) = h(x)`; `y = z − ẑ` (bearing wrapped); `S = H P Hᵀ + R`;
  `K = P Hᵀ S⁻¹`; `x ← x + K y`; `P ← (I − K H) P`.
- One-point initiation: `x=[zx,zy,0,0]`, `P=blockdiag(R_pos, σ_v²I₂)`.

**Assumptions.** `update` is called after `predict` advanced the state to
`z.time`; non-positive `dt` predict is a no-op; small dense matrices so a direct
`S⁻¹` is acceptable; Gaussian noise.

**Rationale.** Standard EKF; the simplest estimator that handles the nonlinear
range/bearing models while remaining cheap and pluggable behind `IEstimator`.

**Ways to improve / test next.** Joseph-form covariance update for numerical
stability; outlier/innovation gating before update; UKF/IMM swap-in via the
`IEstimator` port; two-point velocity initiation.

## 4. Unscented Kalman Filter (`UkfEstimator`)

**Math.** Standard scaled UKF. For state dim `n` and parameters `α, β, κ`:
`scale = α²(n+κ)` (computed directly to avoid catastrophic cancellation when
`α` is small); `λ = scale − n`. Sigma points: `χ₀ = x̂`,
`χᵢ = x̂ ± (√(scale·P))ᵢ` (Cholesky columns).
Weights: `Wm₀ = λ/scale`, `Wc₀ = λ/scale + (1−α²+β)`, others `1/(2·scale)`.

- **Predict:** propagate each `χᵢ` through `F(dt)`; reconstruct mean
  `x̂ = Σ Wmᵢ χᵢ'` and covariance `P = Σ Wcᵢ (χᵢ'−x̂)(χᵢ'−x̂)ᵀ + Q(dt)`.
- **Update:** re-build sigma points from posterior-predict; propagate through
  `h(x)`; reconstruct predicted measurement mean `ẑ = Σ Wmᵢ Zᵢ`, innovation
  covariance `S = Σ Wcᵢ (Zᵢ−ẑ)(Zᵢ−ẑ)ᵀ + R` (bearing residuals wrapped),
  cross-cov `Pxz = Σ Wcᵢ (χᵢ−x̂)(Zᵢ−ẑ)ᵀ`. `K = Pxz S⁻¹`;
  `x ← x + K (z − ẑ)` (residual angle-wrapped); `P ← P − K S Kᵀ`.
- **Initiate:** same one-point initiator as the EKF.

**Assumptions.** `P` is positive-definite (Cholesky succeeds); linear motion
model (`F(dt)·x`); `h(x)` continuous over the sigma spread; the unscented
predicted measurement mean `ẑ` is used as-is for the innovation — under
strong nonlinearity (e.g. very-short-range range/bearing with large prior
position uncertainty) `ẑ` differs from `h(x̂)` by a second-moment bias,
which is exactly the correction the UKF buys you over the EKF.

**Rationale.** Captures mean / covariance through `h(x)` more accurately than
the EKF's first-order linearization, at the cost of `2n+1` evaluations of
`h` per update. Verified correct by **exact agreement with the EKF on linear
predict AND linear update** at 1e-9 tolerance — the canonical sanity check.

**Ways to improve / test next.** Square-root UKF (propagates `S = √P` for
numerical stability); iterated UKF; circular-statistics bearing mean when
sigma spread is wide; a scenario the EKF visibly degrades on (short-range
bearing-only) to put a number on the UKF advantage.

**Numerical note.** Computing `scale = n + λ` from `λ = α²(n+κ) − n` loses
precision when `α` is tiny (the cancellation in `n + (α²(n+κ) − n)` produces
errors of order `n·ε`, breaking the weights). The implementation computes
`scale = α²(n+κ)` directly and recovers `λ = scale − n` from there.

**Measured behaviour on the current scenario suite.** Bit-identical to the
EKF (to four decimal places of mean OSPA) on every existing scenario, because
all of them use Position2D measurements (linear `h`) where UKF and Kalman
posteriors coincide by construction. The UKF advantage will only be visible
once a materially nonlinear scenario exists (short-range range/bearing,
bearing-only, rapid range-rate). See `docs/algorithms/evaluation-log.md` for
the table of measurements.

## 5. Particle Filter (`ParticleFilterEstimator`)

**Math.** Bootstrap (sequential importance resampling) with `N` weighted
particles `{xⁱ, wⁱ}`. State and motion model as for the EKF.

- **Initiate:** sample `xⁱ ~ N(μ₀, P₀)` from the same one-point Gaussian
  initiator as EKF / UKF; `wⁱ = 1/N`. Project ensemble → `(track.state,
  track.covariance)`.
- **Predict:** `xⁱ ← F(dt)·xⁱ + ηⁱ`, `ηⁱ ~ N(0, Q(dt))` via Cholesky
  `Q = L Lᵀ`. Weights unchanged. Project ensemble → carrier.
- **Update:** `log wⁱ ← log wⁱ − ½ yⁱᵀ R⁻¹ yⁱ`, `yⁱ = z − h(xⁱ)`
  (bearing wrapped to (−π, π] via the shared `measurementResidual` helper).
  Normalize via log-sum-exp: `wⁱ ∝ exp(log wⁱ − max log w)`, then `w /= Σ w`.
  Compute `ESS = 1 / Σ (wⁱ)²`. If `ESS < N/2`, **systematic resampling**
  with a single uniform draw `u ∈ [0, 1/N)`. Project ensemble → carrier.
- **Carrier projection:** `x̂ = Σ wⁱ xⁱ`, `P̂ = Σ wⁱ (xⁱ − x̂)(xⁱ − x̂)ᵀ`.

**Assumptions.** Process noise `Q(dt)` is positive-semidefinite; when `Q` is
singular (e.g. `q = 0`) the Cholesky branch is skipped and predict collapses
to a deterministic `F·x`. Measurement noise covariance `R` is
positive-definite (used inverted in the log-likelihood; a sensor reporting
zero-diagonal `R` makes the inverse explode but the degenerate-weights
guard catches it and resets the ensemble to uniform — defensive only, not a
correctness fallback). The initial covariance must be PD; if not, `initiate`
returns a Track with the Gaussian carrier set and *no* particles
(`particles.cols() == 0`), and subsequent `predict` / `update` early-return.
Determinism requires that the call order against a freshly-seeded estimator
be deterministic — the scenario harness guarantees this. Pinned by the
`DeterministicForSameSeed` test (two PFs seeded identically and driven
through the same initiate/predict/update sequence produce bit-identical
particles and weights).

**Rationale.** First estimator that can in principle represent non-Gaussian
posteriors (multimodal range/bearing fusion, bearing-only flow before range
converges). Chosen as the **second** comparison after the UKF because (a) it
requires exactly the same nonlinear measurement model wiring as the UKF,
(b) it trivially handles non-Gaussian priors that an IMM cannot represent at
all, (c) projecting to a Gaussian carrier keeps the pipeline (gating,
association, sinks) estimator-agnostic. Stored as
`(Eigen::MatrixXd, Eigen::VectorXd)` on `Track` itself — colocation gives
clean ownership semantics (no side map, no leak on track deletion).

**Ways to improve / test next.** (1) Auxiliary or marginalized particle
filters that use the measurement to bias the predict step — reduces particle
count needed for sharp likelihoods. (2) Stratified or residual resampling
instead of systematic — lower variance in some regimes. (3) Particle
diversity injection (regularized PF) to recover from over-confident
posteriors. (4) Adaptive `N` based on observed ESS — most updates do not
need 1000 particles. (5) Bearing-only scenario (no range channel) where
the PF's multimodal representation should definitively dominate the
EKF / UKF. (6) Multi-seed Monte-Carlo sweep over `N ∈ {200, 500, 1000,
2000}` to plot the cost / accuracy frontier. (7) Refactor `MeasurementModels`
to expose a `z_pred`-only path so `update` does not allocate a throwaway
Jacobian `H` per particle.

**Measured behaviour on the current scenario suite.** On unimodal
range/bearing passes the PF carries Monte-Carlo variance with no offsetting
analytical advantage and lands slightly *behind* both the EKF and the UKF.
See `docs/algorithms/evaluation-log.md` for numbers — the PF's theoretical
advantage (representing non-Gaussian / multimodal posteriors) is **not**
exercised by the current scenarios; a bearing-only or pre-range-convergence
scenario is the prerequisite for a fair comparison.

## 6. Interacting Multiple Model (`ImmEstimator`)

**Math.** K modes, each running an EKF over its own motion model in a
unified 5-state space `[px, py, vx, vy, ω]`. Fixed `K×K` Markov transition
matrix π; mode probabilities `μⱼ` carried per track.

Per cycle:

- **Mixing (in `predict`).** With the dt-scaled TPM `π(dt) = π^dt`
  (π is the **one-second** TPM; matrix fractional power, equivalent to
  `expm(logm(π)·dt)` for the diagonally-dominant chains used in
  tracking): `cⱼ = Σᵢ π(dt)[i][j]·μᵢ`,
  `μᵢⱼ = π(dt)[i][j]·μᵢ / cⱼ`,
  `x̂₀ⱼ = Σᵢ μᵢⱼ·xᵢ`,
  `P̂₀ⱼ = Σᵢ μᵢⱼ·(Pᵢ + (xᵢ − x̂₀ⱼ)(xᵢ − x̂₀ⱼ)ᵀ)`.
  `predict` then advances `μ ← c` (the predicted mode prior);
  `update`/`softUpdate` consume it directly, so the TPM is applied
  exactly once per predict, scaled by dt — never once more per
  measurement. Rationale: per-call application ties the mode mixing
  rate to the measurement cadence — at the AutoFerry replay's ~16
  scans/s the modes washed to the stationary distribution between
  every measurement. `π^dt` makes prediction a semigroup (two 0.5 s
  predicts ≡ one 1 s predict) and keeps the mixing rate a property of
  the configuration. At the 1 Hz synthetic cadence this is
  bit-identical to the legacy per-call behaviour.
- **Per-mode prediction.** `xⱼ ← Fⱼ(dt)·x̂₀ⱼ`,
  `Pⱼ ← Fⱼ·P̂₀ⱼ·Fⱼᵀ + Qⱼ(dt)`.
  For `CoordinatedTurn`, `Fⱼ(dt)` is evaluated at the mixed-prior `ωⱼ`.
- **Per-mode update (EKF).** `Sⱼ = H Pⱼ Hᵀ + R`, `Kⱼ = Pⱼ Hᵀ Sⱼ⁻¹`,
  `xⱼ ← xⱼ + Kⱼ y`, `Pⱼ ← (I − Kⱼ H) Pⱼ`. Mode likelihood
  `Λⱼ = N(y; 0, Sⱼ) ∝ |Sⱼ|^{−½} · exp(−½ yᵀ Sⱼ⁻¹ y)`.
- **Mode-probability update.** `μⱼ ← cⱼ·Λⱼ / Σₖ (cₖ·Λₖ)` (log-sum-exp).
- **Output projection.** `x = Σⱼ μⱼ·xⱼ`,
  `P = Σⱼ μⱼ·(Pⱼ + (xⱼ − x)(xⱼ − x)ᵀ)`.

**Assumptions.** Unified state dimension across all K models (5-state).
`CoordinatedTurn` is evaluated at the current `ω` estimate (not iteratively
re-linearized through `ω`). Mixing happens inside `predict`, which also
advances the mode probabilities to the predicted prior. Transition
matrix π is time-invariant, chosen by the user, and interpreted as the
TPM at dt = 1 s; it must be embeddable (fractional powers real and
non-negative — true for the diagonally-dominant matrices used here;
non-embeddable chains fall back to per-call application).

**Rationale.** Single-model filters (EKF/UKF/PF over CV) lag through
maneuvers because `Q_CV` does not represent a turn — they widen their
covariance but never adapt their predicted dynamics. IMM keeps a separate
filter per hypothesis and weighs them by data. The unified-5-state design
avoids heterogeneous-IMM dimension bookkeeping at negligible cost.

**Known limitation: position-only measurements + EKF backend → no mode
discrimination.** With `Position2D` measurements alone, `H` has zero in
the `ω` column, so `ω` is unobservable by the linearized update. Both CV
and CT modes settle on `ω_mean ≈ 0`, making their predicted positions
nearly identical, so their likelihoods are nearly identical and the mode
probability is driven entirely by the transition-matrix prior (which
slightly favors whichever mode has the larger self-loop). The IMM-2-mode
configuration **does not** outperform single-mode CV on the current
maneuvering scenario for this reason — confirmed empirically (see
evaluation log).

Three known fixes for this limitation, in order of increasing change:

1. **Prescribed-rate three-mode IMM.** `CV + CT(+ω̂) + CT(−ω̂)`. The two
   CT modes don't have to *discover* the turn rate — they only have to
   recognize a turn matches their prescribed rate. This is the classic
   maritime IMM-3 setup and works with position-only measurements.
2. **UKF per mode.** Sigma points propagate `ω` uncertainty through the
   nonlinear F, so even an initially unknown ω gets a position-domain
   spread that differentiates the modes' likelihoods.
3. **Velocity-bearing measurements.** Add an `H` row that observes velocity
   or heading directly; the CT and CV modes' velocity predictions differ
   visibly and the likelihood ratio shifts.

**Resolution of the limitation: prescribed-rate three-mode IMM.**
Implemented via `PrescribedTurn(omega_const, q_a, q_omega)`, which fixes
ω at construction time and uses it in every `transitionMatrix(dt)` query
(state's ω component is ignored by F but remains in the 5-d state for
unified IMM mixing). The classic maritime configuration is
`{CV5State, PrescribedTurn(+ω̂), PrescribedTurn(−ω̂)}` with a uniform
initial mixture and a transition matrix that lets the modes interconvert
freely (e.g. `[[0.90,0.05,0.05],[0.10,0.85,0.05],[0.10,0.05,0.85]]`).
The CT modes don't have to *discover* the turn rate — each one is
committed to a known rate, so the moment the target turns at that rate,
its likelihood dominates and the mode probability shifts. **Measured: 7.4%
OSPA reduction** vs the CV baseline on the standard maneuvering scenario
(see evaluation log).

**Ways to improve / test next.** (1) UKF backend per mode (sigma-point
propagation through CT lets a single free-ω CT mode work without
prescribing rates). (2) Adaptive transition matrix π. (3) Multi-seed
sweep over turn rate × duration × noise to characterize when IMM-3
dominates. (4) Bank of prescribed rates for a wider maneuver envelope
(±0.1, ±0.2, ±0.5 rad/s).

**Measured behaviour.** See `docs/algorithms/evaluation-log.md`.
