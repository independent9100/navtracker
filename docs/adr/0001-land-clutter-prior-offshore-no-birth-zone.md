# ADR 0001 — Land-clutter prior keeps a near-shore no-birth zone

- **Status:** Accepted — **amended 2026-07-02** (A2 adopted, obstacle-scoped; see below); **partially superseded by ADR-0003 (2026-07-12)** — the 50 m no-birth zone is narrowed to 25 m on the deployable config `imm_cv_ct_pmbm_coverage_land_ivgate`; the `gate == target` mechanism and the philos shore-clutter rationale below stand.
- **Date:** 2026-06-30
- **Deciders:** navtracker maintainers
- **Related:** `docs/algorithms/synthetic-clutter-bench.md`, `docs/algorithms/pmbm-design.md` §10 (land clutter prior), `docs/algorithms/evaluation-log.md` (2026-06-30, Project E; 2026-07-02, R1), `docs/superpowers/specs/2026-06-30-pmbm-land-clutter-prior-design.md`, `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md` (R1)

## Amendment (2026-07-02) — A2 adopted, scoped to static-obstacle composition

Static-branch review ticket **R1** implemented **A2** (apply the phantom-birth
floor to the *pre-suppression* existence) — but **scoped to births where a static
obstacle contributes**, not unconditionally. A clean philos A/B (eval-log
2026-07-02, R1) showed that applying A2 to the **land-only** case *reopens exactly
this no-birth zone* and regresses `imm_cv_ct_pmbm_coverage_land` (card_err
+6.9 → +40.15, gospa 73.1 → 106.9) — the same failure this ADR rejected "A1" for.

So the decision below is **narrowed, not reversed**: the land-only near-shore
no-birth zone under `coverage_land` is **kept**. What A2 fixes is the *additional*
hard-drop created when a soft static-obstacle keep-clear buffer composes with land
(review ticket R1's bug): a real vessel near a **charted obstacle** now births
(with a tiny suppressed existence that accumulates on re-detection) instead of
being silently killed. `imm_cv_ct_pmbm_land` (floor 0.05 < target) — the
recommended general/coastal config — is byte-identical to before either way; only
`coverage_land` (floor == target == 0.1) has the land-only zone, and it is
unchanged. The pure land-only anchored-vessel case remains the open **A3**
(sensor-aware) work below.

## Context

The PMBM tracker's `imm_cv_ct_pmbm_coverage_land` configuration adds a **land
clutter prior** (`ILandModel` / `CoastlineModel`) that suppresses new-target
births on or near the shore. It exists to fight a real failure on the philos
(Boston Harbor) replay: ~185 stationary near-shore radar returns were spawning
phantom tracks. The prior collapses that over-count — `card_err +107.9 → +6.9`,
`gospa_false 23750 → 3550`, `gospa 153.6 → 73.1` — making it the first honest,
no-crutch PMBM config competitive with MHT on philos.

The prior is a **soft signed-distance ramp** `c(d)` over distance `d` to the
nearest shore (d<0 inland): `c = clamp((W_off − d)/(W_off + W_in), 0, 1)` with
`W_off = W_in = 50 m`. A birth candidate's intensity is scaled by `1 − c` (and
hard-dropped well inland). Separately, PMBM applies a **phantom-birth gate**:
a new Bernoulli is discarded when its existence `r_new < min_new_bernoulli_existence`.

In `coverage_land`, **both** `birth_existence_target` and
`min_new_bernoulli_existence` equal **0.1**. The synthetic shore-clutter bench
(Project E, 2026-06-30) added a deliberate validator: a *real* vessel near the
shore. It surfaced — and quantified — a consequence of that equality:

> Because the gate equals the birth target, **any** soft suppression (`c > 0`)
> pushes `r_new` below the gate. Worked example at 10 m offshore: `c = 0.4`,
> scale `0.6`, `r_new = (0.6·0.1)/(0.6·0.1 + 0.9) = 0.0625 < 0.1` → the birth is
> dropped every scan. The soft ramp therefore behaves as a **hard cliff** across
> the entire offshore band: the region within `offshore_halfwidth_m` (50 m) of
> shore is a **no-birth zone**. A vessel inside it never initiates a track under
> `coverage_land`.

This did not show up on philos because that dataset's ground-truth vessels are
all far enough offshore that `c ≈ 0` at their positions; only a perfect-truth
synthetic vessel placed inside the band exposed it.

## Decision

**Keep `min_new_bernoulli_existence = 0.1` (equal to `birth_existence_target`) in
`imm_cv_ct_pmbm_coverage_land`, and accept the <50 m offshore no-birth zone as a
documented limitation of that opt-in config.**

We explicitly **reject** lowering the gate to revive near-shore births.

## Why (rationale)

We measured the obvious fix — call it **A1**: decouple the gate below the target
(`min_new_bernoulli_existence` 0.1 → 0.05) so a softly-suppressed real near-shore
birth (0.0625) clears it. On the synthetic bench A1 works (the near-shore target
is revived). But on the **philos real-data guard it regressed materially**:

| `coverage_land` floor | philos gospa | card_err | gospa_false |
|---|---:|---:|---:|
| **0.10 (kept)** | **73.1** | **+6.9** | **3550** |
| 0.05 (A1) | 100.0 | +36.2 | 9000 |

Lowering the gate re-admits the philos near-shore **water** clutter that only the
higher gate killed (on-*land* clutter is hard-gated regardless of the floor; the
offshore-ramp residual is what the gate caught). That is roughly **a third of the
land model's deployment value** thrown away to protect a case we rarely encounter.

The land prior's entire reason to exist is the philos shore-clutter win. A change
that halves that win to gain near-shore birth sensitivity is a bad trade for this
system: **near-land operation is rare**, whereas suppressing shore clutter in
harbour approaches is the whole point. So we keep the gate and accept the zone.

## Consequences

**Positive**
- The philos real-data win is preserved intact (gospa 73.1, card_err +6.9).
- Behaviour is unchanged for every existing config (the prior is opt-in; default
  configs never touch it).
- The boundary is now explicit and measured, not latent.

**Negative (accepted)**
- Under `coverage_land`, a genuine vessel within 50 m of shore (or near a pier)
  will not initiate a track. Operators using this config near land must know this.
- The Project E validator was reframed accordingly: it now places the real vessel
  60 m offshore (just outside the band) and verifies the prior does **not**
  collaterally suppress legitimate traffic *outside* the zone — which it does not.

## Alternatives considered

- **A1 — decouple the gate (0.1 → 0.05).** Rejected: regresses philos as above.
- **Reduce `offshore_halfwidth_m`** so `c → 0` closer to shore. Not pursued: the
  ramp width is shared with the philos `CoastlineModel`, so narrowing it changes
  the real-data behaviour we are trying to protect, and it only shrinks the zone
  rather than removing the cliff.
- **A2 — a principled fix (left open).** Apply the phantom-birth gate to the
  *pre-suppression* existence, or use a persistence-based birth so a repeatedly
  re-detected real near-shore target ramps past the gate while one-shot clutter
  does not. This could protect genuine near-shore vessels *without* re-admitting
  water clutter. It is a real code change with its own risk and needs its own
  spike + measurement; it is recorded as future work in
  `docs/algorithms/synthetic-clutter-bench.md` §4, not done here.

- **A3 — sensor-aware suppression (the strongest principled fix; needs more
  sensors).** The no-birth zone exists because the prior is **sensor-agnostic**:
  it scales down *any* birth near shore. But shore clutter is overwhelmingly a
  **radar** artifact — fixed structures reflect X-band returns; a camera (EO/IR)
  or a cooperative AIS report near shore is strong evidence of a *real* vessel,
  not clutter. So the right discriminator is the sensor, and that requires the
  extra sensors to be present. Two sub-approaches:
  1. *Sensor-typed prior:* do not apply (or sharply reduce) the land scale for
     births seeded by clutter-free sensors (camera, AIS). A radar-only return
     near shore stays suppressed; a camera/AIS return is allowed to birth. This
     reuses the existing coverage/visibility channel-kind machinery
     (cooperative vs surveillance, ADR-adjacent to `pmbm-design.md` §9).
  2. *Cross-sensor corroboration:* let a near-shore birth corroborated by two
     sensors at the same location override the prior.

  **Important caveat (why this is not free, and why it isn't the current fix):**
  under today's adaptive-birth scheme, `r_new` is pinned to `birth_existence_target`
  *before* the land scale and is algebraically independent of how many sensors
  detected the target (that decoupling from λ_C is by design — see
  `pmbm-design.md` §10). So simply adding a camera does **not** automatically lift
  a near-shore birth past the gate; the *suppression rule itself* must be made
  sensor-aware. Cameras also carry their own near-shore issues (pier/background
  objects, no native range without fusion). A3 is therefore the most promising
  direction once EO/IR or AIS coverage is available near shore, but it is a
  deliberate design change, not an emergent benefit of more sensors.

## Revisit when

- A deployment needs reliable tracking of vessels operating within ~50 m of
  shore under the coverage/land model — then prioritise the A2 spike.
- The land prior is made non-opt-in (canonical) — the no-birth zone would then
  affect more workloads and the trade must be re-evaluated.
