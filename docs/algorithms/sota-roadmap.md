# 2025 SOTA Roadmap

> **Point-in-time snapshot (dated 2026-06-07) — largely superseded.** The
> "Bottom line" and top-3 future investments below (score/existence-probability
> track lifecycle, JIPDA, VB/Schmidt-robust filtering) have since been
> substantially delivered: the IPDA/VIMM existence-probability lifecycle is the
> default (since 2026-06-11 — see [improvement-backlog.md](improvement-backlog.md)),
> and the Random-Finite-Set branch is built out in
> [pmbm-design.md](pmbm-design.md). Treat this document as the historical
> forward-look that motivated that work, not as the current backlog. For the live
> state of what is done vs. open, read [improvement-backlog.md](improvement-backlog.md)
> and [pmbm-design.md](pmbm-design.md). The content below is preserved unchanged
> for provenance.

Forward-looking literature review for navtracker's algorithmic stack. Documents
specific improvements drawn from the 2018–2025 model-based tracking literature
(deep-learning approaches excluded) that go beyond the per-component
"Ways to improve / test next" lists already in `estimation.md`,
`association.md`, and `pipeline.md`. Follows the project documentation
standard: Math / Assumptions / Rationale / Sources.

The baseline against which everything below is compared is the current
implementation as captured in `evaluation-log.md`: EKF / scaled UKF / bootstrap
SIR PF / 3-mode prescribed-turn IMM; GNN / classical-enumeration JPDA /
TOMHT with N-scan trunk merge; M-of-N consecutive-count `TrackManager`;
scalar gyro-bias KF with five observation kinds; CPA evaluator with
hysteresis.

**Assessment.** The current stack is broadly 2015–2018 state-of-the-practice
for shipboard fusion. The existing "ways to improve" lists across the
algorithm docs cover most of 2018–2022 SOTA (Murty k-best, JIPDA,
sqrt-UKF, Joseph form, OOSM retrodiction, free-ω CT via UKF). What is
missing from 2023–2025 model-based literature falls in three buckets:

1. Score- / existence-probability-based track lifecycle replacing M-of-N.
2. The Random Finite Set family (JIPDA → trajectory-PMBM) which is now the
   textbook SOTA for multi-target tracking with native existence semantics.
3. Outlier-robust / VB-adaptive Bayesian filtering (Student-t + VB-AKF +
   Schmidt-KF for residual biases) — the principled answer to "I don't
   know R, my sensors have heavy tails, my biases are not fully observed".

The five highest-ROI improvements, ranked, are below. Each is a discrete
change that fits the existing port surface (`IEstimator`,
`IDataAssociator`, `TrackManager`, `HeadingBiasEstimator`) without
violating the hexagonal-architecture invariants.

---

## 1. Score-based (LLR / SPRT) track lifecycle

**Math.** Per-track log-likelihood-ratio score, updated each scan:

- On association: `L(k) = L(k−1) + ln[ P_D · p(z|x̂) / (λ_C · V_gate) ]`.
- On miss:        `L(k) = L(k−1) + ln(1 − P_D)`.

Confirmation and deletion thresholds derived from Wald's SPRT:

- `T_C = ln((1 − β) / α)` confirms at desired false-track rate `α` and
  missed-confirmation rate `β`.
- `T_D = ln(β / (1 − α))` deletes symmetrically.

**Replaces.** `core/tracking/TrackManager` consecutive-count
`hits ≥ confirm_hits` / `misses ≥ delete_misses` state machine.

**Assumptions.** Per-scan Gaussian likelihood `p(z|x̂) = N(z; ẑ, S)` (the
MHT path already computes this for its leaf scores); known per-sensor
`P_D` and clutter density `λ_C`; gate volume `V_gate` from the χ² gate
threshold.

**Rationale over M-of-N.** Consecutive-count `N` is the same in calm and in
heavy sea-state — it cannot trade confirmation latency against clutter
density. SPRT is provably the minimum-expected-sample-size test for a
given (α, β) (Wald 1947); in sea clutter where `λ_C` is the dominant
nuisance, an SPRT/LLR lifecycle adapts automatically and gives the operator
a calibrated rather than tuned false-track rate. The MHT leaf-score
formula already in `MhtTracker` is identical to the SPRT score, so the
LLR lifecycle composes for free when MHT is selected; for GNN / JPDA the
score is built per-scan from the gating residual.

**How JIPDA fits.** Under JIPDA (improvement 2) the per-track existence
probability `r(k)` is the logit transform of the LLR. The same lifecycle
implementation services both score-based and probability-based
formulations — pick one parameterisation, the other comes for free.

**Expected benefit.** Blackman (2004) reports 2–3× reduction in false
confirmations at equal detection rate vs M-of-N on clutter-dominated
radar. Score-based confirmation is what the navtracker docs already call
out as "score-based confirmation/deletion as a first-class alternative
to M-of-N" — this entry promotes it from wishlist to ranked top item.

**Scenarios to validate.** Heavy-clutter ARPA tracking with intermittent
low-SNR detections; sea state ≥ 4 synthetic clutter or replayed real-radar
data (see DLR / Fowdur 2021 in the validation roadmap).

**Sources.**
- Blackman, "Multiple Hypothesis Tracking for Multiple Target Tracking",
  *IEEE AES Magazine* 19(1), 2004.
- Blackman & Popoli, *Design and Analysis of Modern Tracking Systems*,
  Artech 1999, Ch. 6.
- Wald, *Sequential Analysis*, Wiley 1947.
- "SPRT-Based Track Confirmation and Rejection",
  https://ieeexplore.ieee.org/document/1020914/

---

## 2. JIPDA + Set-JPDA upgrade of the JPDA path

**Math (JIPDA).** Augment each track with existence probability `r(k)`:

- Prediction:  `r(k|k−1) = p_s · r(k−1|k−1)`.
- After observing `m_k` measurements with feasible-joint marginals
  `β(j, t)` and `β_0(t)` as already computed in `JpdaAssociator`,
  the existence update is the standard Mušicki–Evans recursion that
  collapses the no-target hypothesis with the data-likelihood-weighted
  target hypothesis.

**Math (Set-JPDA).** Standard JPDA's PDAF soft update,
`x ← x + K · Σ β_jt y_jt`, is *not* permutation-invariant when two
tracks gate on the same measurement set: both posteriors get pulled to
the centroid of the measurement cluster (track coalescence). SJPDA
reformulates the soft update on the joint state random set, then
re-labels for output — the result is the same likelihood spread without
the coalescence pathology.

**Replaces / augments.** `JpdaAssociator` (existing path); the existence
probability is the natural input to the LLR lifecycle (improvement 1) and
the `enter_probability` / `exit_probability` thresholds in
`CpaEvaluator`.

**Assumptions.** Same as the current JPDA implementation (Gaussian
likelihoods, full enumeration tractable at cluster scale); per-track
survival probability `p_s` configured per sensor / scenario.

**Rationale.** The current JPDA already shows 64% fewer ID switches and
60% fewer ghost tracks than GNN on the documented clutter-crossing
scenario (`evaluation-log.md`). The remaining failure modes are
track coalescence on close-spaced parallel targets — exactly the SJPDA
fix — and lack of a calibrated existence probability for downstream
lifecycle and CPA hysteresis — exactly the JIPDA fix. The two together
upgrade the JPDA branch from "soft data association" to "soft data
association with native existence semantics and no coalescence".

**Expected benefit.** SJPDA eliminates parallel-track coalescence
(Svensson et al. 2011, Fig. 4). JIPDA gives a probability `r(k) ∈ [0,1]`
that calibrates lifecycle decisions and feeds `CpaEvaluator` hysteresis
correctly instead of via tuned thresholds.

**Scenarios to validate.** Two ships in a TSS lane at similar speeds and
within gate overlap; AIS-dropout with bias estimator on (existence
probability should decay gracefully through the gap).

**Sources.**
- Mušicki & Evans, "Joint Integrated Probabilistic Data Association (JIPDA)",
  *IEEE TAES* 40(3), 2004,
  https://ieeexplore.ieee.org/document/1337482/
- Svensson, Svensson, Guerriero & Willett, "Set JPDA Filter for Multitarget
  Tracking", *IEEE TSP* 59(10), 2011,
  https://publications.lib.chalmers.se/records/fulltext/146503/local_146503.pdf
- Mušicki, La Scala & Morelande, "Linear Multitarget Integrated Probabilistic
  Data Association" (LMIPDA), 2007.

---

## 3. Iterated EKF for bearing updates + Cubature KF replacing scaled UKF

**Math (IEKF).** One Gauss-Newton step on the MAP cost per measurement:

```
x⁽⁰⁾ = x_pred
for i = 0..N_iter:
  (ẑ, H) = h(x⁽ⁱ⁾)         # re-linearised at current iterate
  y = z − ẑ                  # bearing wrap as today
  S = H P_pred Hᵀ + R
  K = P_pred Hᵀ S⁻¹
  x⁽ⁱ⁺¹⁾ = x_pred + K (y − H (x_pred − x⁽ⁱ⁾))
  if ‖Δx‖ < ε: break
P = (I − K H) P_pred
```

Typically 1–3 iterations suffice. Damped (Levenberg–Marquardt) variant
adds robustness against poor predicts.

**Math (CKF).** Third-order spherical–radial cubature: `2n` equally-weighted
points `χᵢ = x̂ ± √n · (√P)ᵢ`, all weights `w = 1/(2n)`. No `α/β/κ`
tuning; no negative weights; predicted and updated covariance
unconditionally sign-definite.

**Replaces / augments.**
- IEKF is a per-iteration loop inside `EkfEstimator::update`, gated by a
  config knob `n_iter` (default 1 = current behaviour). The Joseph form
  already on the wishlist composes naturally.
- CKF is a sibling `CkfEstimator` behind `IEstimator`. It is *not* a
  drop-in replacement for the scaled UKF: keep the UKF available, default
  to CKF for new deployments and for the IMM-CT backend where the UKF's
  negative-weight numerical fragility is most acute.

**Assumptions.** IEKF: `h` differentiable at every iterate (range > 0,
which is already guarded). CKF: `P` positive-definite (Cholesky succeeds).
For both, the measurement noise covariance `R` is positive-definite.

**Rationale.** Two distinct gains.

- *IEKF*: standard EKF linearises `h` once at the predict. For bearing-only
  EO/IR updates at short range the linearisation error is exactly the
  failure mode the scaled UKF was added to address; IEKF closes the same
  gap with one Gauss-Newton step and no sigma-point machinery. Bell &
  Cathey (1993) showed IEKF = Gauss-Newton MAP; in practice 1–2 iterations
  cut linearisation error by 1–2 orders of magnitude at close range.
- *CKF*: the scaled UKF in `UkfEstimator` already required a numerical-
  stability patch (computing `scale = α²(n+κ)` directly to avoid
  cancellation when `α` is tiny). CKF removes the entire failure mode by
  construction — `2n` equally-weighted positive points, mathematically a
  UKF with `α=1, β=0, κ=0` but expressed without the cancellation-prone
  weight algebra. Comparable or better RMSE than UKF at lower wall-clock
  cost has been reported in multiple radar studies (Arasaratnam & Haykin
  2009; *Remote Sensing* 16(22), 2024).

**Scenarios to validate.** Close-quarters EO/IR-only handover; fast crossing
target inside 500 m; a deliberately short-range range/bearing scenario
designed to make the EKF visibly degrade (currently flagged in
`estimation.md` §4 as the missing test).

**Sources.**
- Bell & Cathey, "The iterated Kalman filter update as a Gauss–Newton
  method", *IEEE TAC* 38(2), 1993.
- Arasaratnam & Haykin, "Cubature Kalman Filters", *IEEE TAC* 54(6), 2009.
- "A Monte Carlo-Based Iterative EKF for Bearings-Only Tracking of Sea
  Targets", *Sensors* 2024,
  https://pmc.ncbi.nlm.nih.gov/articles/PMC11014230/
- "Observability-Based Gaussian Sum CKF for 3-D Target Tracking",
  *Remote Sensing* 16(22), 2024, https://www.mdpi.com/2072-4292/16/22/4172

---

## 4. Trajectory-PMBM as a third `IDataAssociator`

**Execution plan:** [`docs/superpowers/plans/2026-06-07-pmbm-integration-plan.md`](../superpowers/plans/2026-06-07-pmbm-integration-plan.md)
— phased roadmap (Murty K-best → GM-PMBM → IMM-PMBM → TPMBM → GOSPA),
effort estimates, porting pitfalls. Read that for the *how*; the entry
below is the literature *why*.


**Math.** The Poisson Multi-Bernoulli Mixture filter represents the
multi-object posterior as the union of (a) a Poisson point process over
hypothetically-existing-but-never-detected targets and (b) a mixture
of multi-Bernoulli components over detected targets, each Bernoulli
carrying existence probability and a Gaussian (or Gaussian-mixture)
spatial density. The recursion is conjugate under the standard
point-target model with Poisson birth. Trajectory-PMBM (García-Fernández
et al. 2020) extends the state to a *trajectory* — the sequence of states
from a target's birth to its current scan — so the filter outputs labelled
trajectories directly, matching navtracker's `TrackOutput` data model.

**Truncation.** Hypothesis count is bounded by Murty's k-best assignment
or, in the modern formulation, by a Gibbs sampler over global association
events (Vo, Vo & Hoang 2017; Trezza, Bucci & Vo 2022). Same truncation
algorithm services both PMBM and the planned MHT global-non-conflict step
in `MhtTracker` — they share an implementation.

**Adds.** A third concrete `IDataAssociator` (alongside `GnnAssociator`,
`JpdaAssociator`) wrapping a Bernoulli-component manager and the
shared truncation. Reuses the existing `IEstimator` backend per Bernoulli
component.

**Assumptions.** Standard point-target model (one measurement per target
per scan, conditionally independent given assignment). Poisson birth
intensity configurable per sensor. EKF/UKF/CKF backend for the per-
component spatial density.

**Rationale.** GNN commits per scan; JPDA spreads update mass per scan
but commits to track identities; MHT defers commitment over N scans
without modelling birth/death principally; PMBM is the modern conjugate
generalisation that handles birth, death, missed detections, and clutter
in a single closed-form recursion *and* carries native existence
probabilities. Maritime evaluations (*Sci. Rep.* 2025; *Signal Processing*
2024) show PMBM matching or beating MHT and δ-GLMB on clutter-heavy radar
scenarios at lower implementation cost than labelled GLMB variants. The
trajectory formulation, specifically, eliminates the label-management
machinery that δ-GLMB requires.

**Why PMBM over δ-GLMB.** Both are RFS conjugate priors with native
existence probabilities. PMBM has a simpler propagation step (no labelled
GLMB component algebra), benchmarks comparably, and the trajectory-PMBM
variant produces labelled output directly without the labelled-prior
machinery.

**Scenarios to validate.** Mixed AIS/ARPA scenario with un-cooperative
targets entering and leaving the volume; sustained clutter with target
birth and death events; the existing JPDA-vs-MHT comparison scenario
extended with birth/death.

**Sources.**
- García-Fernández, Williams, Granström & Svensson, "Poisson Multi-Bernoulli
  Mixtures for Sets of Trajectories", arXiv:1912.08718, 2020.
- García-Fernández, Williams, Granström & Svensson, "Poisson Multi-Bernoulli
  Mixture Filter: Direct Derivation and Implementation", *IEEE TAES* 54(4),
  2018.
- Vo, Vo & Hoang, "An Efficient Implementation of the GLMB Filter",
  arXiv:1606.08350.
- Trezza, Bucci & Vo, "Linear-Complexity Gibbs Sampling for GLMB Filtering",
  arXiv:2211.16041, 2022.
- "Multiple-Model Trajectory PMBM for Maneuvering Objects", *Sci. Rep.* 2025,
  https://www.nature.com/articles/s41598-025-28096-1
- "Novel MTT Based on PMBM Filter for High-Clutter Maritime Communications",
  2025, https://www.researchgate.net/publication/388292642
- Mahler, *Statistical Multisource-Multitarget Information Fusion*, Artech
  2007/2014.

---

## 5. Variational-Bayes Student-t adaptive measurement model + Schmidt-KF for residual bias

**Math (Student-t innovations).** Replace the Gaussian measurement
likelihood `N(z; ẑ, R)` with a multivariate Student-t with degrees of
freedom `ν`:

```
p(z | x) = t_ν(z; ẑ, R)
```

The robust EKF update collapses to a Gaussian update with a *down-weighted*
gain when the squared Mahalanobis innovation exceeds the `ν`-dependent
threshold — outliers are softly rejected without a hard gate.

**Math (VB-adaptive R).** Place an Inverse-Wishart prior on `R`:

```
p(R) = IW(R; Λ, ν_R)
```

The variational-Bayes recursion (Särkkä & Nummenmaa 2009) alternates one
state update at the current `E[R⁻¹]` with one IW update at the residual
moment, in closed form per measurement.

**Math (Schmidt-KF treatment of bias).** Partition the state into
estimated `x` and considered bias `b`. Schmidt-KF carries the cross-
covariance `P_xb` and uses it in the Kalman gain *without* updating `b`
from each measurement:

```
S = H_x P_xx H_xᵀ + H_x P_xb H_bᵀ + H_b P_bxᵀ H_xᵀ + H_b P_bb H_bᵀ + R
K_x = (P_xx H_xᵀ + P_xb H_bᵀ) S⁻¹
x ← x + K_x y
P_xx ← P_xx − K_x (H_x P_xx + H_b P_bxᵀ)
P_xb ← P_xb − K_x (H_x P_xb + H_b P_bb)
# P_bb unchanged — bias not updated
```

**Augments.**
- `MeasurementModels.hpp` gains a t-likelihood variant; `EkfEstimator`
  (and `CkfEstimator` once added) gain a VB inner loop on `R`.
- The bias estimated by `HeadingBiasEstimator` is exposed to the tracker
  as a *considered* state in `Tracker::update` — its variance correctly
  inflates per-track covariance without re-estimating bias inside the
  tracker.

**Assumptions.** t-distribution `ν` chosen per sensor (typical: `ν = 4`
for ARPA with sea-clutter heavy tails; `ν = 6` for EO/IR; Gaussian limit
as `ν → ∞`). Inverse-Wishart hyperparameters from a calibration pass on
historical data. `HeadingBiasEstimator` exposes posterior mean and
covariance of the bias state.

**Rationale.** Three previously-separate problems collapse to one
principled recursion. (a) ARPA split-detections and EO/IR spurious
bearings produce heavy-tailed innovations — exactly what Student-t was
designed to absorb without a hard gate. (b) Per-sensor `R` is in fact
unknown and time-varying (weather, sea state, range-dependent radar
noise) — VB-AKF is the field-standard online answer, recursive and
stable. (c) `HeadingBiasEstimator` has converged but residual bias
variance still matters for collision-risk CPA inflation — Schmidt-KF
treatment ensures the per-track covariance is *honest* about the bias
residual without double-counting it.

**Why this matters for real-world deployment.** Synthetic Gaussian tests
do not exercise heavy tails, unknown `R`, or unmodelled bias residuals.
The first time navtracker meets real radar in sea-state ≥ 4 the Gaussian
assumption breaks; the first time it meets a sensor whose `R` was
specified by the OEM under nominal conditions and is wrong by 2× under
real ones, an adaptive recursion is what keeps innovations consistent;
the first time the heading-bias estimator settles but is still residually
uncertain, downstream CPA decisions need to know.

**Scenarios to validate.** Synthetic Gaussian + 5%–10% contamination
from a heavy-tail mixture (validates t-filter); long-duration replay where
true `R` is held constant for the synthetic baseline and time-varying for
the test (validates VB-AKF); bias-on / bias-off CPA enter-rate comparison
(validates Schmidt-KF treatment).

**Sources.**
- Roth, Özkan & Gustafsson, "A Student's t filter for heavy-tailed process
  and measurement noise", *ICASSP* 2013.
- Roth, Özkan & Gustafsson, "Robust Bayesian Filtering and Smoothing Using
  Student's t Distribution", arXiv:1703.02428.
- Huang, Zhang, Wu & Chambers, "A novel robust Student's t-based Kalman
  filter", *IEEE TAES* 2017.
- Särkkä & Nummenmaa, "Recursive Noise Adaptive Kalman Filtering by
  Variational Bayesian Approximations", *IEEE TAC* 54(3), 2009.
- "Self-Tuning Process Noise in VB-AKF for Target Tracking",
  *Electronics* 12(18), 2023, https://www.mdpi.com/2079-9292/12/18/3887
- Schmidt, "Application of state-space methods to navigation problems",
  *Advances in Control Systems* 3, 1966.
- Woodbury, "Mitigating the effects of residual biases with Schmidt-Kalman
  filtering", 2005, https://ieeexplore.ieee.org/document/1591877/

---

## Honourable mentions

These have real but lower ROI than the top five. Listed with one-line
rationale and pointers; see the per-algorithm "Ways to improve" lists
for the items already captured there.

**Joint sensor registration with N cooperative AIS targets.** When `N ≥ 3`
AIS targets are co-tracked by ARPA, the constant biases
(`Az_bias, Rng_bias, Rng_scale`) are over-determined by linear least
squares per scan. Strictly better than the scalar gyro-bias KF when many
cooperative targets are present; degrades naturally to it when only one is.
Implement as a 2-step EM (E: associate AIS↔ARPA; M: closed-form LS for
biases) inside `AisArpaPairExtractor`.
*Source.* Helmick & Rice, "Removal of alignment errors in an integrated
system of two 3-D sensors", *IEEE TAES* 1993; Li, Jilkov & Mušicki, "A
Survey on Joint Tracking Using EM-Based Techniques", *Information Fusion*
2017.

**Rauch–Tung–Striebel / Unscented-RTS smoother in the replay path.**
Post-processes a logged scenario to produce a smoothed trajectory with
typically 30–50% lower RMSE than the forward-only estimate. Zero cost to
the online path; useful for post-incident reconstruction, ground-truth
generation, and OSPA baselines in the metrics harness.
*Source.* Särkkä, "Unscented Rauch–Tung–Striebel Smoother", *IEEE TAC*
2008, https://users.aalto.fi/~ssarkka/pub/uks-preprint.pdf; Särkkä,
*Bayesian Filtering and Smoothing*, CUP 2013.

**Track-to-track fusion (Covariance Intersection).** Symmetric to
`ITrackSink`: add an `ITrackSource` so navtracker can *consume* another
tracker's outputs. CI as the default fusion rule for tracks of unknown
provenance; Bar-Shalom–Campo full-cross-cov fuse for trusted federated
peers.
*Source.* Julier & Uhlmann, "A non-divergent estimation algorithm in the
presence of unknown correlations", *ACC* 1997; Chen, Chong & Mori, "A
Review of Forty Years of Distributed Estimation", *Fusion* 2018; Noack,
Sijs, Reinhardt & Hanebeck, "Decentralized data fusion with inverse
covariance intersection", *Automatica* 2017.

**Multi-frame / N-scan global assignment via Lagrangian relaxation.**
Proper-name of the global multi-scan assignment that TOMHT currently
lacks (the documented "two trees can share a measurement in one scan"
defect). Implement once; services both MHT global non-conflict and PMBM
truncation.
*Source.* Poore, "Multidimensional Assignment and Multitarget Tracking",
*DIMACS* 1995; Poore & Robertson, *Comp. Opt. Appl.* 1997.

**Feature-aided association.** When AIS reports MMSI + ship type +
dimensions, factor the association cost as
`p(z|x) = p(z_kin|x_kin) · p(z_feat|x_feat)`. Highest leverage point
for AIS-ARPA fusion because ship length/type from AIS is a hard
discriminator between close-spaced ships.
*Source.* Drummond, "Feature-aided tracking", 1999; Bar-Shalom & Blair,
*Multitarget-Multisensor Tracking: Applications and Advances Vol. III*,
ch. 2.

---

## Bottom line

navtracker's current stack reflects ~2015–2018 state-of-the-practice;
its per-component "Ways to improve" lists cover most of 2018–2022 SOTA;
the five items above plus the honourable mentions close the gap to
2025 model-based SOTA. The three highest-impact for real-world
deployment, in order, are:

1. **SPRT/LLR (or JIPDA-existence-probability) track lifecycle** —
   eliminates the "tune N for clutter" failure mode that surfaces the
   first time the system meets sea state ≥ 4.
2. **JIPDA + Set-JPDA upgrade of the JPDA path** — calibrates existence
   probability for downstream CPA and removes the parallel-track
   coalescence pathology that surfaces in dense TSS traffic.
3. **VB-adaptive Student-t measurement model + Schmidt-KF for residual
   bias** — the principled answer to heavy-tailed innovations, unknown
   `R`, and converged-but-not-perfect bias estimates, all of which appear
   on first contact with real sensors and none of which the current
   synthetic test suite exercises.

Trajectory-PMBM and IEKF + CKF are the natural next investments once
the lifecycle, JPDA, and noise-model layers are SOTA.
