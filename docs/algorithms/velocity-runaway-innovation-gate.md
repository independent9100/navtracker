# Velocity-runaway guard — the update-acceptance position-innovation gate

*Backlog #25 Phase 2b. A per-instance, default-OFF kinematic guard in
`PmbmTracker` that stops a track's velocity state from running away on a
gross close-pass mis-association. Intuitive introduction:
`docs/learning/11-gating-gnn-hungarian.md` §"A second gate". Provenance:
`docs/baselines/2026-07-08_b25_localization.md` (H3 diagnosis),
`docs/baselines/2026-07-09_b25_phase2a_probe.md` (offline probe),
`docs/baselines/2026-07-09_b25_phase2b.md` (true-innovation re-probe),
`docs/baselines/2026-07-09_b25_phase2b_stage2.md` (this build's A/B).*

## 1. Math

At a detection update, a Bernoulli with post-predict (pre-update) mean
`x̂⁻` and an assigned position-model measurement `z` (ENU) has **position
innovation**

    ν = z − H x̂⁻ ,   H = [I₂ 0],   ‖ν‖ = position-innovation norm (m).

The guard fires when `‖ν‖ > D_max`. The association is already decided (this
Bernoulli won the assignment for `z`); the update is applied normally, so the
**position** is accepted. The guard then treats the velocity/turn-rate block of
the state — indices `{2,3}` (vx, vy) and, for the 5-state IMM CV/CT model,
index `4` (ω) — of the moment-matched `(x, P)` **and every IMM mode**
`(xᵏ, Pᵏ)` (the IMM predict reads the per-mode ensemble, so treating only the
moment-matched projection would be re-corrupted on the next predict):

- **Deweight** (shipped): raise each treated marginal variance to a floor
  `σ²_floor` (never shrink) and zero that component's cross-covariances,
  `P[i,i] ← max(P[i,i], σ²_floor)`, `P[i,·]=P[·,i]=0` for `i∈{2,3,4}`. The
  velocity **mean is kept**. Subsequent position updates re-estimate velocity
  through the widened gain.
- **Reset**: additionally `x[i] ← 0` for `i∈{2,3,4}` (discard the velocity mean).

`σ²_floor = innov_gate_velocity_var_floor` (default `1e4` (m/s)² ≈ σ 100 m/s, a
wide maritime prior). Existence `r`, aggregated mass, birth, and track id are
**not** functions of `ν` and are never touched.

Default `innov_gate_max_m ≤ 0` ⇒ the branch is skipped ⇒ output byte-identical
to a build without the guard (proven: all 46 states.csv + non-timing metric rows
identical, gate on vs off, and vs the pre-guard binary).

## 2. Assumptions

1. **Position-model measurements.** `ν` in position space needs `z` to be an ENU
   position (`Position2D` / `PositionVelocity2D`). Radar/AIS/lidar are pre-
   projected to ENU by the adapters, so this holds for them; bearing-only
   (`Bearing2D`) carries no position → the guard does not fire on it (norm
   sentinel). Range-bearing not pre-projected would need its own handling
   (none of the current scenarios feed it raw).
2. **A gross position innovation means the wrong return, not a real manoeuvre.**
   A vessel cannot truly jump `D_max` (≥ 200 m) between updates at ≤ 2.5 s
   spacing; at maritime speeds that gap is a mis-association. `D_max` must sit
   above the largest *legitimate* innovation (own-ship-relative motion + sensor
   noise) — the probe measured 0 % false-fire on real workloads at 400 m.
3. **Accepting the position keeps the estimate in the real world.** The winning
   measurement, even if it is the neighbour vessel, is a real return near the
   scene — so the position stays plausible while the velocity re-learns. (The
   watch-item — accepting the neighbour's position can migrate the id onto it —
   is measured, not assumed: see §4.)
4. **Presence, not deletion.** Per ADR 0002 the fix must keep the object present;
   the failure being repaired is the *estimate leaving*, not the track dying.

## 3. Rationale

Backlog #25 localised the close-pass track loss to **H3 — estimator state
divergence**: the velocity state runs away while existence `r` stays high, so the
estimate leaves the gate and the truth goes unassigned across the CPA. Two
offline probes then chose the lever and its shape:

- A **velocity-magnitude** bound was measured and **rejected** (Phase 2a): healthy
  well-positioned tracks carry transient velocity-*state* spikes (159–235 m/s) in
  close-pass ambiguity, so a speed threshold false-fires 2–11 % on honest-truth
  data. The **position innovation** is the clean discriminator (a measurement-
  corrected healthy track's position barely moves even when its velocity state
  overshoots) — 0.06 %/0.00 % false-fire at D 400 m.
- Placement is **update-acceptance**, not an estimator-internal magnitude clamp:
  the runaway is triggered by one oversized *accepted* innovation, and a
  magnitude clamp on the moderate build-up innovations would re-introduce the
  velocity-signal false-fire. The build-up itself is CT-mode-driven (see §4).
- The **action** was picked by measurement, not argument (Stage-2 A/B, 6 dying
  cases, gate = loss-seconds-overlapping-CPA + re-acquire-id count):

  | variant | CPA-overlap loss (s) | total dying loss (s) | re-acquire ids | id-switches (all/dense) |
  |---|---|---|---|---|
  | OFF | 163 | 1366 | 45 | 34 / 30 |
  | reset D200 | 51 | 630 | 9 | 13 / 12 |
  | reset D400 | 299 | 550 | 14 | 17 / 13 |
  | deweight D200 | 179 | 939 | 17 | 9 / 8 |
  | **deweight D400** | **6** | **544** | 10 | 15 / 11 |

  **Deweight @ D_max 400 m wins the safety gate** (CPA-overlap loss 163 → 6 s).
  Reset *stalls* the track (velocity → 0 while the target keeps moving), so it
  stays lost through the CPA; deweight keeps the track moving in roughly the
  right direction with a wide velocity prior, so it re-locks. id-switches **fall**
  vs OFF (no swap regression — the watch-item cleared).

- **No-regression:** philos KEEP + AutoFerry (real) are **byte-identical** with the
  guard on (no accepted innovation there exceeds 400 m); sim_ms mostly improves.
  The guard is kinematic-only, so the miss-P_D existence brake and the birth/λ_C
  invariant are untouched by construction.

## 4. Ways to improve / what to test next

- **The velocity build-up precedes the trigger.** The gate fires on the single
  oversized innovation, but the velocity is already elevated by a *run of moderate
  innovations under CT (turn) mode dominance*. Deweight recovers anyway (wide
  prior + subsequent positions), but a **CT-mode-keyed estimator clamp** (bound
  the per-update velocity/turn-rate change *when the CT mode dominates*, keyed on
  mode weight — NOT on innovation magnitude) is the escalation path if a workload
  is found where deweight does not recover. The CV↔CT mode-thrash signature
  (`imm_mode_weights` in the diag export) is the lead.
- **Coalescence is adjacent.** The dying ids migrate across neighbouring truths at
  the CPA (id 6 tracks truth 152/153 then jumps onto 151; id 7 covers two truths).
  Accepting the neighbour's position can migrate an id — measured here as **no net
  id-switch rise** (34 → 15), but a dedicated **coalescence guard** is the parked
  companion lever (separate ticket) if a workload shows swaps rising.
- **`D_max` band + phantom interaction.** 400 m is the clean point (200 m is
  marginal on false-fire and on CPA-overlap for deweight). The census's 82.5 %
  clutter-born phantom majority is **presence-neutral** under the guard (total
  clutter track-seconds ≈ unchanged: fewer confirm, survivors persist) — the
  guard is not a phantom killer; if phantom suppression is wanted it belongs in
  the birth/existence channel (backlog #23), not here.
- **True innovation vs range-bearing.** If a consumer feeds raw `RangeBearing2D`,
  add the ENU projection for the innovation (the diag/guard currently sentinels
  non-position models).
- **Test next:** a synthetic sustained-close-pass scenario with ground-truth
  velocity (to grade the re-learned velocity RMSE, not just presence); an
  AIS-dropout close pass (does deweight over-widen when the next fix is delayed?).
