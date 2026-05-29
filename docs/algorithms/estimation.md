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
