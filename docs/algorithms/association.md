# Association & Track-Management Algorithms

Follows the project documentation standard: Math / Assumptions / Rationale /
Ways to improve. Cross-reference: design spec sections 7 and 11.3.

## 1. Gating (`mahalanobisDistance`)

**Math.** `(ẑ,H)=h(x)`; innovation `y=z−ẑ` (bearing wrapped); innovation
covariance `S=H P Hᵀ + R`; gating statistic `d²=yᵀ S⁻¹ y` (χ²-distributed with
DOF = measurement dimension under the Gaussian hypothesis).

**Assumptions.** Gaussian innovations; `S` positive-definite (true when `P` and
`R` are PD).

**Rationale.** Standard χ² gate; `d²` doubles as the association cost.

**Ways to improve / test next.** Reuse the computed `S`/inverse in the EKF
update; add the `0.5·ln|S|` term for a proper log-likelihood cost.

## 2. Greedy GNN association (`GnnAssociator`)

**Math.** Cost between a track and a measurement is the gated `d²`. Greedy
assignment: repeatedly select the globally smallest in-gate pair (`d² ≤ gate`),
assign it, remove both from consideration, repeat until no in-gate pair remains.
Unselected tracks and measurements are returned as unmatched.

**Assumptions.** At most one measurement per track per call; the gate is a χ²
quantile for the measurement dimension (e.g. 9.21 ≈ χ²₂ at 0.99); pairwise
independent costs.

**Rationale.** Deterministic and simple; sufficient to validate the fusion
pipeline before investing in optimal assignment (spec D5).

**Ways to improve / test next.** Optimal 2D assignment (Hungarian/auction) so a
locally cheap greedy pick can't block a better global solution; JPDA for
ambiguous clutter; MHT for deferred decisions; MMSI / sensor-track-ID hint
locking before kinematic gating; feature-aided (size/type) costs.

## 3. Track lifecycle (`TrackManager`)

**Math/Logic.** New track: `Tentative`, `hits=1`, `misses=0`. `recordHit`:
`hits++`, `misses=0`; `hits ≥ confirm_hits` → `Confirmed`. `recordMiss`:
`misses++`, `hits=0`; `misses ≥ delete_misses` → deleted (removed); else
`Coasting`. IDs: monotonic `uint64` from 1, never reused.

**Assumptions.** Consecutive-count policy (not sliding window); one hit/miss per
track per cycle; orchestration lives in the pipeline.

**Rationale.** Simplest deterministic M-of-N machine; stable never-reused IDs
satisfy the core identity invariant independent of MMSI (spec D1).

**Ways to improve / test next.** Score-based (log-likelihood-ratio)
confirmation/deletion; sliding-window M-of-N; re-promote Coasting → Confirmed on
re-acquisition.

## 4. Joint Probabilistic Data Association (`JpdaAssociator`)

**Math.** Soft data association via classical joint event enumeration.
M measurements, T tracks. Validation matrix `V` (M×T binary) gates pairs
by Mahalanobis distance. A feasible joint event θ is a function
{1..M} → {0, 1..T} where `θ(j) = 0` means "clutter" and `θ(j) = t` means
"assigned to track t"; no two measurements share a non-zero target.

Joint event weight (un-normalized):

```
w(θ) ∝ λ_C^N_FA · P_D^N_D · (1−P_D)^(T−N_D) · Π_{j: θ(j)≥1} N(z_j; ẑ_{θ(j)}, S_{θ(j)})
```

where N_D = |{j: θ(j) ≥ 1}|, N_FA = M − N_D. Normalize over all feasible
events (log-sum-exp). Marginal:
- `β(j, t) = Σ_{θ: θ(j) = t+1} w(θ)`
- `β_0(t) = Σ_{θ: t not detected} w(θ)`

**PDAF soft update (in `EkfEstimator::softUpdate`):**

- `y_combined = Σ_j β_jt y_jt`
- `x_t ← x_t + K_t y_combined`
- `spread = K_t [Σ_j β_jt y_jt y_jtᵀ − y_combined y_combinedᵀ] K_tᵀ`
- `P_t ← β_0_t P_t + (1−β_0_t)(I − K_t H_t) P_t + spread`

**Assumptions.** Full enumeration is tractable for the cluster sizes the
caller passes in (≤ ~6 tracks × ~6 measurements per scan). Beyond that,
K-best or Markov-chain approximations are needed. `P_D` (detection
probability per track per scan) and `λ_C` (clutter spatial density) are
configuration; they should match the sensor's empirical detection and
clutter characteristics. All gated measurements share a measurement model
(`softUpdate` uses `H`, `R` from the first).

**Per-sensor (P_D, λ_C) — JpdaAssociator multi-sensor mode (backlog
item 8, 2026-06-13).** The scalar `(P_D, λ_C)` is dimensionally incorrect
across mixed sensors: `log p(z|x) − log λ_C` is only dimensionless when
`λ_C` shares the measurement-model's natural units (m⁻² / (m·rad)⁻¹ /
rad⁻¹), and a single scalar cannot satisfy that for a radar + bearing
scan. The per-sensor ctor `JpdaAssociator(gate, ISensorDetectionModel*)`
resolves each measurement's `(P_D, λ_C)` via `model->paramsFor(z)` —
same port the MHT path uses, so a scenario's per-sensor detection table
drives both pipelines identically. Joint-event log-weight becomes

```
log w(θ) = Σⱼ [θ(j)==t+1] · (log P_D[s(j)] + log p(z_j|x_t))
        + Σⱼ [θ(j)==0]   · log λ_C[s(j)]
        + Σₜ [t not detected in θ] · Σ_s ∈ S(θ) log(1 − P_D^s(x_t))
```

where the per-track miss factor is aggregated over distinct `(sensor,
model, source_id)` tuples present in the scan via
`missDetectionProbability(...)` — coverage-conditioned (out-of-range or
out-of-sector charges no miss penalty), same convention as
`TrackTree::branch` in the MHT path. The single-Gaussian fallback path
(no estimator) sees `result.p_d = P_D` of the first measurement when the
batch is homogeneous, otherwise 0 → `ImmEstimator::softUpdate` falls
back to its unnormalized mixture-likelihood proxy. The scalar ctor is
retained unchanged and bit-identical to the legacy code path.

**Rationale.** This is step 1 of the JIPDA upgrade (sota-roadmap.md §2):
per-track existence math needs per-sensor (P_D, λ_C) wired before
existence can be updated correctly under multi-sensor batches. It also
closes the single-hypothesis half of the AutoFerry calibration story —
the MHT path has been honestly multi-sensor since 2026-06-11, JPDA was
silently using one global λ_C on the same scenes.

**Rationale.** GNN commits to a hard pairing per scan; in clutter or
near-crossing geometries it commits wrong and the wrong measurement
contaminates the track, leading to ID switches and ghost tracks from
clutter. JPDA spreads the update across all gated assignments weighted by
their joint probability — no commitment, no contamination. Classical
enumeration is the exact baseline against which cheaper approximations
are compared. **Measured: 64% reduction in ID switches and 60% reduction
in ghost-track count vs GNN on the clutter-crossing scenario** (see
evaluation log).

**Ways to improve / test next.** (1) K-best joint events (Murty / Auction)
for tractable JPDA at larger cluster sizes. (2) JIPDA — also tracks
per-track existence probability, ties in with M-of-N lifecycle.
(3) UKF / PF / IMM `softUpdate` implementations (currently EKF-only;
others fall back to the no-op default). (4) Per-measurement-model
`R`-and-`H` support in `softUpdate` (currently assumes uniform model
across the gated set). (5) MHT as the natural next step — defers
commitment further by maintaining a hypothesis tree. (6) Per-sensor
batch decomposition in `associate(...)` — process each sensor's
measurements as a sequential JPDA scan instead of one joint event
over the union, the textbook multi-sensor fusion. The current
homogeneous-batch shortcut for `result.p_d` makes this almost free for
the common case (single-sensor scans); the gain is on mixed-sensor
scans where today's per-track miss aggregation is exact but the IMM
mixture normalization falls back to the proxy.

**Measured behaviour.** See `docs/algorithms/evaluation-log.md`.

## 5. Multiple Hypothesis Tracking — TOMHT (`MhtTracker`)

**Math.** Track-oriented MHT. Each track owns a `TrackTree` of hypothesis
nodes. Each node carries `(state, covariance, parent, scan_idx, score)`
where score is the cumulative log-likelihood-ratio (LLR).

Per scan:

- **Branch.** For each leaf in each tree, produce one missed-detection
  child and one child per gated measurement
  (`Δscore = log P_D + log N(z; ẑ, S) − log λ_C`, with `(P_D, λ_C)`
  looked up per (sensor, model, source_id) from `ISensorDetectionModel`
  — units of λ_C live in the sensor's measurement space; the
  measurement-resolved overload `paramsFor(z)` is virtual so λ_C can be
  spatially resolved at z's position, see §6). The source_id
  refinement (2026-06-11, backlog item 4) lets two physical sensors
  sharing a `SensorKind` calibrate independently: on AutoFerry the EO
  and IR cameras are both `SensorKind::EoIr` yet measure P_D ≈ 0.73 vs
  0.46 against ground truth. Sources without an exact entry fall back
  to the kind-wide entry, then to the model defaults. The EKF update is
  applied per measurement-assigned child.

  The miss child charges `Δscore = Σ_s log(1 − P_D^s(x))` over the
  *distinct (sensor, model, source) triples present in this scan*,
  where `P_D^s(x)` is coverage-conditioned: 0 beyond the sensor's
  `max_range_m` about its position, and 0 outside its azimuth sector
  (`sector_center_rad` ± `sector_width_rad`/2, ENU math convention,
  default full circle) → zero penalty. The sector is fixed in the ENU
  frame; entries for sensors on rotating platforms must be expressed in
  absolute azimuth (per-measurement sensor attitude is future work).
  Asynchronous multi-sensor rationale: scans
  are per-sensor timestamp groups arriving at the union of all sensor
  rates; a global per-scan `log(1 − P_D)` makes the miss cost
  proportional to the *total event rate* (~16 Hz on AutoFerry) and
  deletes any track its fastest sensor cannot see ≈0.4 s after its
  last hit (measured: ~600 track breaks per AutoFerry scenario, philos
  lifetime 0.02). With per-sensor conditioning the same scenarios run
  at ~65 breaks / lifetime 0.77 under the canonical config.
  The IPDA miss recursion uses the scan's effective
  `P_D = 1 − Π_s (1 − P_D^s(x))`, and the IPDA/VIMM persistence
  parameters are **per-second rates** applied as `π^dt` (1 Hz cadence
  reproduces the classical per-scan recursion exactly).

  Since 2026-06-11 the IPDA + VIMM existence/visibility lifecycle is
  the **default** (`use_ipda_lifecycle = use_visibility = true`):
  confirm/delete read the existence posterior r instead of M-of-N hit
  counts / the raw LLR score. Measured (2026-06-11 baseline, honest
  per-sensor tables everywhere): bit-identical to M-of-N on clean
  synthetics — the lifecycles only diverge where misses are actually
  processed — and decisively better under misses/clutter
  (dense_clutter OSPA 421 → 245, AutoFerry scenario2 breaks
  64.5 → 1.5, lifetime 0.77 → 0.95). M-of-N and SPRT remain available
  as ablations (`use_ipda_lifecycle = false`).

  IPDA confirmation uses **hysteresis**: first confirmation requires
  `r ≥ ipda_confirm_threshold`; once confirmed, the track holds
  Confirmed while `r ≥ ipda_demote_threshold` (< confirm), and after
  demotion must re-cross the full confirm threshold. Rationale: the
  existence posterior is volatile scan-to-scan (one weak/missed scan
  drops r from 0.92 to ~0.5 at P_D 0.9), and a memoryless threshold
  readout turns each shallow dip into a one-scan Tentative hole —
  lifecycle flicker that fragments downstream consumers' picture
  without any genuine doubt about the track. The ever-confirmed flag
  lives on the `TrackTree`; `demote == confirm` reproduces the
  memoryless readout exactly.
- **K_local prune.** Drop the lowest-scoring leaves per tree until at most
  `k_max_leaves` remain.
- **N-scan trunk-merge.** For each tree, find each current leaf's
  scan-(t−N) ancestor. The ancestor with the highest descendant-leaf
  score wins; prune all other branches at that depth and their
  descendants. Commits the past ≥ N scans to a single hypothesis while
  the most recent N scans stay multi-hypothesis.
- **Spawn.** Any measurement not gated to any existing tree's best leaf
  starts a new track tree.
- **Cross-tree duplicate merge.** When two trees' best leaves stay
  within a Bhattacharyya bound (position block, default 1.0) for
  `duplicate_merge_seconds` (default 3.0 s) of sustained stream time,
  the younger tree is retired and the older external id survives
  (ID-stability invariant). Rationale: the global hypothesis only
  enforces *per-scan measurement* exclusivity — with several detections
  of one target per scan (multi-sensor), tree A takes one hit and tree
  B another, so both stay confirmed forever: a permanent +1 cardinality
  error and id flapping in any downstream assignment (~59 residual
  id_switches on AutoFerry scenario2). The streak is **time-based, not
  scan-counted** — a scan-counted streak of 3 is ~0.19 s at AutoFerry's
  16 Hz union rate and merged real vessels passing close (same
  multi-rate lesson as scan-counted M-of-N). The clock resets the
  moment a pair separates, so crossing targets that brush past never
  accumulate it. Set the threshold ≤ 0 to disable.
- **Output.** For each tree, the best-scoring leaf's `(state, covariance)`
  is the externally-visible track.

**Assumptions.** EKF backend (score formula assumes Gaussian likelihood);
state space matches `IEstimator::initiate`. Track identity is stable:
each tree gets a monotonic external id at spawn that is never reused.
Tracks are independent across trees: this first-cut MHT does **not**
enforce global non-conflict (the same measurement can in principle
contribute to two trees' winning leaves within one scan). The
K-best-global / Murty's-algorithm extension is captured as future work.
Tracks are dropped when their best leaf's score falls below
`score_delete_threshold`, or — under the default IPDA lifecycle —
when every leaf's existence probability falls below
`ipda_delete_threshold`.

**Rationale.** GNN commits per scan and stays wrong; JPDA spreads update
mass per scan and stays uncommitted; MHT carries multiple full-history
hypotheses across N scans before any commitment. The structural advantage
shows up when the optimal assignment is ambiguous *at the current scan*
but becomes clear after seeing 2-3 more scans — exactly the
crossing-with-dropout case. **Measured: 7x lower OSPA than JPDA, zero ID
switches vs JPDA's 2, correct final track count (2 vs JPDA's 3)** on the
documented scenario.

**Ways to improve / test next.** (1) ~~K-best global non-conflict~~
DONE — Murty K-best with Score-Δ protected alternatives. (2) ~~Score /
existence-based confirmation/deletion~~ DONE — IPDA/VIMM lifecycle is
the default since 2026-06-11. (3) UKF / PF backends for the per-leaf
predict + update + likelihood (IMM is in). (4) ~~Cross-tree track
merging~~ DONE 2026-06-11 — see the duplicate-merge pass above; next
refinement would be merging *hypotheses* (mixture fusion) instead of
retiring the younger tree. (5) Sensitivity sweep over (dropout length,
N_scan, closest-approach distance). (6) Track-to-track bias-aware
distance in the merge bound once inter-sensor registration biases are
modelled (improvement-backlog §9). (7) ~~Source-keyed detection
entries + azimuth sectors~~ DONE 2026-06-11 (backlog item 4) — but the
companion lesson is recorded in the evaluation log: feeding the
*measured* urban-channel camera clutter rate into the uniform-λ score
collapses urban lifetime, because shoreline returns are persistent
structure, not Poisson clutter. The spatial clutter map (§6, backlog
item 5) is the vehicle for that. (8) ~~Bearing-driven identity churn
(scenario5)~~ RE-DIAGNOSED 2026-06-12: the ~91 switches are a
*duplicate-birth conveyor* (bearing-carried tracks turn overconfident,
sparse radar returns escape the gate and birth replacements every
2–4 s), not solve-level bearing swaps — measured via
`share_ambiguous_bearings` (bit-identical on sc5) and birth forensics
(45/48 near-truth confirmations with a live track < 50 m). Three
opt-in knobs exist (`share_ambiguous_bearings`,
`DetectionParams::gate_threshold`, `gate_recapture_tau_s` — see
MhtTracker::Config docs); none is default because the root cause is
filter overconfidence (sc5 NEES 77.6 vs ~2 — backlog item 12), and
gate-level remedies recapture without correcting (the Kalman gain
shares the bad P). See evaluation-log 2026-06-12.

**Configuration choices for the documented scenario.** P_D = 0.9,
λ_C = 1e-4, gate = 9.0 (χ²₂ at 0.99), N_scan = 3, K_max_leaves = 5,
score_delete_threshold = −15.0. N_scan = 3 covers the 4-scan dropout
because the leaf depth grows by 1 per processed scan (no growth during
the dropout window where there are no measurements to process), so the
trunk extends through the gap.

**Measured behaviour.** See `docs/algorithms/evaluation-log.md`.

## 6. Spatial clutter map (`ClutterMapSensorDetectionModel`)

Backlog item 5 (2026-06-12). A decorator over the fixed per-sensor
detection table that makes λ_C a function of *where* the measurement
is, learned online from the scan stream.

**Math.** Per (SensorKind, MeasurementModel) the model keeps a sparse
grid of cells; cell c holds an EWMA estimate `r_c` of "clutter
evidence per scan landing in c":

```
touch at time t with weighted count n = Σ_j w_j over returns in c:
  w   = 1 − exp(−(t − t_last)/τ)        (first touch: Δt = prior_dt_s)
  r_c ← (1 − w)·r_c + w·n
```

Per-return clutter weights are labeled by `MhtTracker` from the
**chosen global hypothesis** (2026-06-12, second iteration): a return
claimed by some tree's selected hit leaf — or that birthed a
still-alive tree this scan — carries `w_j = 1 − r` of that hypothesis'
existence; an unclaimed return carries 1.0. Clutter-born trees keep
low existence, so the clutter signal survives, while returns claimed
by confident tracks contribute ≈ 0. (The first iteration used the
binary birth-gate proxy — "gated to no tree" — which charged every
birthing return at full weight: the clean-scene "birth self-poisoning"
tax.) A cell is touched on every scan in which *any* return lands in
it; confidently-claimed traffic contributes ≈ 0 and drags the cell
toward zero. New cells are seeded with the table baseline
(`r = λ_table · A`), so an untouched map reads back the table exactly.
Query — the virtual `paramsFor(z)` the TrackTree score already calls:

```
λ_c  = r_c / A                  (A = cell_size² m², or cell width rad)
λ(z) = interpolation of λ_c at z's position
λ    = clamp(λ(z), λ_table·min_ratio, λ_table·max_ratio)
```

Position sensors (Position2D / PositionVelocity2D) use a 2-D ENU grid
(default 50 m pitch, bilinear interpolation over the 4 surrounding cell
centers, λ in m⁻²). Bearing sensors (Bearing2D) have a 1-D circular
grid over absolute ENU azimuth (default 5° pitch, linear interpolation
with ±π wraparound, λ in rad⁻¹) that is **OFF by default**
(`enable_bearing_map`, see the measured death spiral below).
RangeBearing2D ((m·rad)⁻¹ space) has no map yet and passes through.
P_D, `max_range_m`, and sector fields always come from the wrapped
table — only `clutter_intensity` is position-resolved. The EWMA weight
is **time-based (τ seconds), never scan-counted** (multi-rate lesson:
10 scans is 0.6 s for a 16 Hz camera and 100 s for AIS).

**Measured negative result — the bearing-map death spiral
(2026-06-12).** Bearings cannot initiate tracks, so a real target whose
track has lapsed (occlusion, score dip) keeps feeding "unassociated"
bearings at its own azimuth. The map then raises λ exactly where the
target is, every subsequent camera hit scores worse, re-confirmation is
suppressed, and the suppression self-reinforces. Per-sub-map ablation
on AutoFerry (canonical config, scenario × {fixed, full map, position
map only, bearing map only}): the position map alone is
lifetime-neutral on all scenarios while the bearing map alone
reproduces the full collapse — sc17 lifetime 0.90 → 0.28, sc5
0.91 → 0.34, sc2 0.96 → 0.74 (baseline
`2026-06-12_clutter_map_bearing_spiral`). The bearing map's apparent
OSPA gains came from suppressing *true* tracks alongside false ones.
**Existence-weighted hypothesis labeling does not fix it** — measured
strictly worse (sc17 0.13, sc5 0.10): a coasting track's claimed
bearings carry weight `1 − r` precisely while `r` is low, so the map
feeds on the target during exactly the occlusions the track must
survive. The spiral is structural until either tracks can be born
from bearings or the weight distinguishes "low-existence target" from
"no target" (e.g. visibility-conditioned weights, or excluding
returns claimed by *any* live hypothesis regardless of r). Opt-in via
`enable_bearing_map`.

**Assumptions.** (1) "Gated to no existing track" is a usable clutter
proxy — persistent structure that births its own clutter track becomes
"associated" one scan later, so the proxy *undercounts* stable shoreline
structure for position sensors; bearings cannot initiate tracks and keep
counting. (2) Clutter fields move slowly relative to τ. (3) Indexing
bearing clutter by absolute azimuth from ownship is useful while ownship
moves slowly relative to τ.

**Rationale.** Both real datasets show the same failure mode: harbour
clutter is persistent *structured* shoreline return, not uniform
Poisson, and feeding its ML-fitted rate into the uniform-λ score
collapsed true-track lifetime (measured 2026-06-11, AutoFerry urban
channel; same caveat recorded for philos Boston-harbour radar). A map
keeps the calibrated baseline where nothing has been learned and raises
λ only where false returns actually recur — the clutter-map CFAR idea
transplanted into the MHT score. A decorator (rather than a new table
type) keeps the fixed path bit-identical when unwrapped and the hot
path untouched. Clamping as *ratios* of the local table baseline keeps
the band dimensionally sane across m⁻² and rad⁻¹ sensors
simultaneously.

**Ways to improve / test next.** (1) ~~Label clutter from the
global-hypothesis association instead of the birth-gate proxy~~ DONE
2026-06-12 (existence-weighted claims, see Math) — it removed the
birth tax but, against the original hypothesis, did NOT make the
bearing map safe (measured worse; see above). (2) A bearing weight
that distinguishes "low-existence target" from "no target":
visibility-conditioned weights (a VIMM-coasting track's claimed
bearings should weigh ~0, not 1 − r), or a hard zero for returns
claimed by any live hypothesis. (3) Per-source maps (EO vs IR) if
per-source clutter measurably differs. (4) A range×bearing
product-space map for RangeBearing2D sensors. (5) Forgetting toward
the prior for cells unvisited ≫ τ (currently they keep their last
estimate). (6) Feed the map into JPDA β computation once backlog item
8 (JPDA per-sensor parity) lands.

**Measured behaviour.** Bench ablation `imm_cv_ct_mht_cmap` (canonical
stack + map) vs `imm_cv_ct_mht` — see `docs/algorithms/evaluation-log.md`
2026-06-12 entry.
