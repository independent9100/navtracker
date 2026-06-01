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

**Ways to improve / test next.** UKF for stronger nonlinearity or bearing-only
geometry; particle filter for multimodal cases (bearing-only before range
converges); per-sensor `R` calibration from data.

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
