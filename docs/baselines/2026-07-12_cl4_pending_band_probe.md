# Cl-4 endgame — the user's two-threshold scheme: shape-collapse proof + pending-band census

**Branch `cl4-pending-band-probe` (unmerged). Measurement only, zero shipped
behaviour change.** Origin: the Cl-4 endgame cliff-reprice
(`docs/baselines/2026-07-11_cl4_cliff_price_list.md`, Parts 1+2, merged 7080949)
and the user's 2026-07-12 proposal of a generalized near-shore scheme: instead
of one ramp + one fixed bar, distance-dependent accept/reject thresholds — and,
crucially, **two** thresholds, implying a middle "pending" zone between reject
and accept. Ticket:
`docs/superpowers/plans/2026-07-12-cl4-pending-band-probe-ticket.md`.

Two halves: (Step 1) close the pure-shape question with a proof + empirical
check; (Step 2–3) measure the genuinely new half — the pending band — against
the existing Pareto front. Instrument: the sanctioned env-knob method
(byte-identical when unset) + the offline census tool `tools/cl4_a3_census.cpp
--mode pending`. Suite green at 0 skips (see end).

## TL;DR

- **Step 1 — shape collapses to a boundary (CONFIRMED).** Under adaptive birth
  every candidate's pre-suppression existence is pinned to `birth_existence_target
  = 0.1` (§3.2.2 invariant). Distance-to-shore is the only per-return input, so
  `r_new(d)` is monotone and any one-shot ramp×bar shape reduces to a single
  admit boundary `d*`. Two deliberately non-linear shapes (quadratic; two-segment
  strict-then-lenient) built with the same `d*=25 m` as `W25/f0.10` reproduce
  that cell: **env-2 revival identical (8/8, gospa 13.376), philos phantom bill
  within seed noise** (linear==quad byte-identical; seg +0.10 card_err = 1/50th
  of one floor step). A shape cannot beat the measured front.
- **Step 2 — the pending band uses TIME, which distance cannot.** In-band births
  confirm only after re-detection on ≥K scans. Census of the philos OFFSHORE
  in-band population (345 returns; the 1361 inland returns are hard-gated and out
  of the denominator) chained by re-detection: the transient water clutter dies
  in pending, a small persistent-offshore residual survives.
- **Step 3 — it matches the front, it does not beat it.** The pending band is a
  smooth latency-for-phantoms curve. At K=1 it *is* A2 (unconditional
  floor-lowering); every K≥2 drops below A2; it reaches the one-shot `W25/f0.10`
  phantom cost at **K≈5–8** (chain-gate-sensitive), where env-2 revival latency
  is **~7–17 s** and it covers the **full 0–50 m band** the one-shot's `d≥25`
  boundary cannot. The surviving offshore phantoms there are few (single digits)
  and dominated by persistent stationary reflectors — the A2 failure mode in
  miniature, but small because most philos in-band persistence is inland
  (already hard-gated). **No recommendation — the price decision is the user's.**

---

## Step 1 — shape-collapse verification

### The structural claim (why shape is a red herring)

Adaptive birth chooses `λ_birth = (r*/(1−r*))·λ_C` so that the pre-suppression
existence of *every* birth candidate equals `r* = birth_existence_target = 0.1`,
independent of clutter intensity (`pmbm-design.md` §3.2.2, the λ_C-cancellation
invariant). There is **no per-return score variation**. The only thing that
varies across the near-shore birth population is the position, and the land model
collapses that to a single scalar `c_land = clutterPrior(pos) ∈ [0,1]`, a monotone
function of the signed distance-to-shore `d`.

The materialised birth existence (land-only config, no static obstacle, so the
gate reference equals the suppressed value — `PmbmTracker.hpp` `applyBirthPriors`
`else` branch) is

```
r_new(c) = 0.1·(1−c) / (0.1·(1−c) + 0.9),     admit iff r_new ≥ floor.
```

`r_new` is monotone decreasing in `c` and `c` is monotone increasing in the ramp
shape, which is monotone decreasing in `d`. So `r_new(d)` is monotone and the
admit test `r_new ≥ floor` is a **single boundary `d*`**: admit iff `d ≥ d*`. Any
monotone ramp shape × any monotone bar(d) picks some `d*` — a point the 2-D
`W_off × floor` surface already swept. **A shape cannot reach an operating point
off that boundary locus.**

At `W25/f0.10` the algebra pins the boundary to the *open-water* value:
`r_new ≥ 0.10 ⟺ 0.9·(1−c) ≥ 0.9 ⟺ c = 0 ⟺ d ≥ W_off = 25 m`. Every admitted birth
therefore sits at `c=0`, `r_new=0.1` — the shape is **fully erased** at admission.
This is the clean cell to test the collapse on: two shapes with the same
zero-crossing must give a byte-identical admitted-birth set and existence.

### Instrument

`PMBM_RAMP_SHAPE ∈ {linear (default=identity), quad, seg}` — a research-only env
lever wired in `core/benchmark/Sweep.cpp` as a monotone remap `h:[0,1]→[0,1]` over
the shipped `clutterPrior` (`ShapedLandModel` decorator), with `h(0)=0`, `h(1)=1`
so the open-water and hard-gate endpoints — and thus the admit boundary at
`floor ≥ 0.1` — are preserved. UNSET ⇒ the base model verbatim, byte-identical,
no wrapper allocated. Shapes:

- `quad`: `h(c) = c²` (convex; lenient mid-band).
- `seg`: two-segment strict-then-lenient with a knee at `c=0.5`:
  `(0→0), (0.5→0.8), (1→1)`.

Both cross zero exactly where linear does, so all three share `d*=25 m`.

### Result (all at `PMBM_OFFSHORE_HALFWIDTH_M=25 PMBM_MIN_NEW_BERN=0.10`, seed 1)

| shape | env-2 tracked | env-2 gospa | env-1 gospa | philos card_err | philos gospa | philos gospa_false |
|---|---:|---:|---:|---:|---:|---:|
| linear (shipped ramp) | 8/8 | 13.3758 | 16.5720 | **+17.3500** | 84.5751 | 5640 |
| quad   | 8/8 | 13.3758 | 16.5720 | **+17.3500** | 84.5751 | 5640 |
| seg    | 8/8 | 13.3758 | 16.5720 | **+17.4500** | 84.6700 | 5660 |

Per-env-2-scenario gospa is identical to 5 decimals across all three shapes
(sc13 14.3486, sc16 10.5754, sc17 15.5755, sc22 13.0039).

**Reading.** `linear` and `quad` are **byte-identical**. `seg` differs by +0.10
card_err (+0.6 %), +0.09 gospa (+0.1 %), +20 gospa_false (+0.35 %) — and env-2 is
identical to it too. This is exactly the predicted second-order residue: a *gated*
in-band candidate (d<25, dropped) still adds `log(rho_total_suppressed)` to its
child hypothesis weight (`PmbmTracker.cpp:1139`), and `rho_total_suppressed`
depends on the shape (`c` differs for d<25). It perturbs hypothesis-weight
bookkeeping but **cannot move the (revival, phantom) operating point**, which is
fixed by the admit boundary `d*=25` all three share. For scale: one floor step
(0.01) moves philos card_err ~+5; the seg residue is 1/50th of that — far inside
seed noise. **Collapse confirmed; the shape family is closed.** (Had the shapes
diverged materially, that would have been the stop-and-report — the collapse
argument would have a hole. They did not.)

*Note the linear row reproduces the merged front cell `W25/f0.10` exactly (philos
+17.35, env-2 8/8 gospa 13.376), so the Step-1 instrument is consistent with the
Part-2 surface.*

---

## Step 2 — the pending-band census

Scheme under test: an in-band birth candidate (0 < d < 50 m, charted coastline
only) is neither killed nor confirmed at first sight — it confirms only after
re-detection on **≥ K scans** within a short window. This is the one thing
distance cannot encode: **time**. Offline census, no tracker code.

### philos (the guard) — how much offshore clutter survives pending

The philos OFFSHORE in-band radar population (`0 ≤ d < 50 m`) is chained by
re-detection (Phase-1b single-link NN chainer; a chain = re-detections of one
object across scans). **The 1361 inland returns (`d<0`) are hard-gated regardless
and kept out of the denominator; 345 offshore returns remain.** The philos radar
clip is ~10 Hz over ~19 s (199 scans). Reported for a loose gate (`r_chain=25 m`,
the Phase-1b default) and a tight gate (`r_chain=15 m`) as a bound.

`chains ≥ K` = births that would confirm through the pending band. K=1 = no
persistence = the A2 anchor (unconditional floor-lowering) in this instrument.

**r_chain = 25 m (loose):**

| K | chains≥K | 0–10 m | 10–25 m | 25–50 m | stationary | moving | proj tracks/scan |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 241 | 96 | 73 | 72 | 218 | 23 | 14.30 |
| 2 | 52 | 23 | 15 | 14 | 29 | 23 | 13.09 |
| 3 | 23 | 12 | 5 | 6 | 10 | 13 | 8.28 |
| 5 | 9 | 5 | 1 | 3 | 4 | 5 | 4.32 |
| 8 | 1 | 0 | 1 | 0 | 0 | 1 | 0.93 |

**r_chain = 15 m (tight):**

| K | chains≥K | 0–10 m | 10–25 m | 25–50 m | stationary | moving | proj tracks/scan |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 274 | 111 | 84 | 79 | 269 | 5 | 10.29 |
| 2 | 44 | 19 | 13 | 12 | 39 | 5 | 8.91 |
| 3 | 13 | 7 | 4 | 2 | 8 | 5 | 3.63 |
| 5 | 4 | 2 | 0 | 2 | 2 | 2 | 1.69 |
| 8 | 0 | 0 | 0 | 0 | 0 | 0 | 0.00 |

`stationary`/`moving` split at net chain displacement 15 m. `proj tracks/scan` =
a confirmed chain is present from its K-th detection to its last, summed over
confirmed chains ÷ 199 scans (a coast-to-last projection, stated as such).

**Reading.** The transient water clutter dies fast: 345 offshore returns → 241/274
raw chains, but only **52/44 reach K=2 and 23/13 reach K=3**. The survivors are
increasingly the *persistent* returns — at K≥3 the stationary fraction is the
moored-object / stable-reflector population that passes any pure-persistence band
(the A2 failure mode). But it is **small in absolute count** (single digits at
K≥3–5) and, decisively, **most philos in-band persistence is inland** (1361
hard-gated returns) — the offshore persistent population the band must fight is
modest. Some "moving" survivors may be real vessels missing from the sparse philos
AIS truth (the `gospa_false` caveat), i.e. returns that *should* be tracked. The
count is **gate-sensitive** (~2× between r15 and r25): treat it as a band.

### env-2 (the revival) — latency, and does every target reach K

Each env-2 vessel re-detects on essentially every scan (97–204 distinct-scan
re-detections inside its first in-band window across the four scenarios), so
**all 8 targets reach every K up to 8** — none fails to revive. The cost is pure
latency. Aggregated across the 8 targets (autoferry_scenario{13,16,17,22}):

| K | latency min | median | max |
|---:|---:|---:|---:|
| 2 | 1.7 s | 3.4 s | 6.7 s |
| 3 | 3.4 s | 5.0 s | 8.4 s |
| 5 | 6.7 s | 10.1 s | 16.8 s |
| 8 | 12.6 s | 15.1 s | 21.8 s |

### harbor + env-1 (no collateral, by construction)

The band only exists where a charted coastline is loaded. `harbor_complete_truth`
is chart-free (no coastline path → the ramp is inert, `inBand` false everywhere),
and env-1 vessels are open-water (`d ≥ 50 m`, `c=0`, never suppressed). The
census confirms zero in-band offshore returns / zero in-band truth for both →
the pending band never engages them. (Cross-checked against Step-1: env-1 gospa
16.572 is unchanged across all shapes, and the Part-2 surface already showed
harbor 9.53 / env-1 16.57 invariant to every floor and W_off.)

---

## Step 3 — score against the existing front

Two reference costs from the merged Part-2 surface: the best one-shot cell
**`W25/f0.10` = +10.45 tracks/scan** (card_err delta over the no-revival baseline
+6.90, strip-confined, 8/8 env-2) and the **A2 anchor = +36.15 card_err**
(unconditional floor-lowering at 0.05).

The census projection and the measured card_err are **different instruments**.
The census `--mode pending` prints a *same-instrument* one-shot `W25/f0.10`
reference (the `d≥25 m` offshore chains, no persistence): **3.59 tracks/scan
(r25) / 2.91 (r15)**, against the externally measured **+10.45**. So the chain
projection undercounts absolute cost ~3× (it does not model track drift /
hypothesis multiplication / coasting). Two consequences:

1. **Relative in-instrument comparison is robust** (both the one-shot and the
   pending band are scaled by the same ~3× factor, as both are persistent
   near-shore returns).
2. **Internal consistency check:** K=1 (whole-offshore, no persistence) ×
   the calibration ≈ 10.3–14.3 × (10.45/2.91…3.59) ≈ **37–42 ≈ A2 (+36.15)** —
   K=1 *is* A2, and both instruments agree it is.

**The combined picture (in-instrument tracks/scan; one-shot ref in bold):**

| scheme | r_chain 25 | r_chain 15 | env-2 latency | reach |
|---|---:|---:|---|---|
| A2 (floor 0.05, measured) | — | — | 0 s | ~all offshore |
| pending K=1 (= A2 in-instrument) | 14.30 | 10.29 | 0 s | 0–50 m |
| pending K=2 | 13.09 | 8.91 | ≤6.7 s | 0–50 m |
| pending K=3 | 8.28 | 3.63 | ≤8.4 s | 0–50 m |
| pending K=5 | 4.32 | 1.69 | ≤16.8 s | 0–50 m |
| pending K=8 | 0.93 | 0.00 | ≤21.8 s | 0–50 m |
| **one-shot W25/f0.10 (ref)** | **3.59** | **2.91** | **0 s** | **25–50 m only** |

**Reading.**
- **vs A2:** the pending band is strictly better for any K≥2 (A2 *is* K=1). It
  must "sit far below A2 to be real" (ticket) — it does from K≈3 on (K=3
  in-instrument 8.28/3.63 vs K=1 14.3/10.3), and by phantom-seed *count* the
  margin is large (K=3: 23/13 chains vs 241/274).
- **vs the front:** the pending band reaches the one-shot's in-instrument phantom
  cost at **K≈5–8 for the loose gate (r25) and K≈4–5 for the tight gate (r15)**.
  At that K the whole 0–50 m band is revived (the one-shot only reaches 25–50 m)
  and env-2 latency is ~7–17 s.
- **Why it only matches, not beats:** by phantom-*seed* count the pending band
  (K≥3: 13–23 chains) is well below the one-shot's 72–79 seeds — but that is
  misleading. Most one-shot seeds are transient and would die in the tracker's own
  M-of-N confirmation, so the one-shot's real +10.45 already comes from its
  *persistent* 25–50 m returns. Both schemes ultimately confirm only persistent
  returns; the pending band's net difference is that it **adds the persistent
  0–25 m offshore returns** (~6 chains at K=5, r25) as phantoms **in exchange for
  reviving vessels in the 0–25 m band** the one-shot's `d≥25` boundary excludes.
- **Placement:** the pending band's phantoms are near-shore/in-strip by
  construction (offshore in-band births, like `W25`); it does not spill into open
  water the way the floor dial (`W50/f0.08`) does.

**Bottom line (no recommendation):** the pending band is a genuine *third*
operating point, not a free lunch. It trades ~7–17 s of env-2 latency for the
one-shot front's phantom cost while extending revival to the full 0–50 m band,
with a small residual of persistent offshore reflectors (single digits) that any
pure-persistence scheme admits. Whether the wider reach + latency + a real
tracker build is worth it over the one-shot `W25/f0.10` (which needs only the
already-merged `PMBM_OFFSHORE_HALFWIDTH_M` lever set as default) is the user's
call. The known risk the ticket asked to measure — persistent stationary offshore
returns — is present but small, because philos in-band persistence is
overwhelmingly inland and already hard-gated.

## Caveats (projection limits)

1. **Offline projection.** `proj tracks/scan` is a coast-to-last estimate on the
   birth-candidate re-detection structure; it undercounts absolute card_err ~3×
   (calibrated against the one-shot reference). Relative comparisons are robust;
   absolute numbers are not. A real answer needs the pending logic in the tracker.
2. **Chain-gate sensitivity.** Survivor counts and the crossover K move ~2×
   between r_chain 15 and 25 m — reported as a bound, not a point.
3. **Structure vs truth-missing vessel.** The census cannot separate a moored
   reflector from a real slow/stopped vessel absent from the sparse philos AIS
   truth; "moving" survivors may be legitimate tracks.
4. **One geography.** philos (Boston) is the only charted-shore guard with dense
   offshore clutter; the persistent-offshore fraction is a property of this shore.

## Reproduce

```
# Step 1 — shape collapse (byte-identical when PMBM_RAMP_SHAPE unset):
for SHAPE in linear quad seg; do
  SH=""; [ "$SHAPE" != linear ] && SH="PMBM_RAMP_SHAPE=$SHAPE"
  env PMBM_OFFSHORE_HALFWIDTH_M=25 PMBM_MIN_NEW_BERN=0.10 $SH \
    ./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_coverage_land_ivgate \
    --scenario-filter autoferry_scenario --seeds 1 --run-id af_$SHAPE --out <dir>
  env PMBM_OFFSHORE_HALFWIDTH_M=25 PMBM_MIN_NEW_BERN=0.10 $SH \
    ./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_coverage_land_ivgate \
    --scenario-eq philos --seeds 1 --run-id ph_$SHAPE --out <dir>
done

# Step 2/3 — pending-band census (offline, no bench):
./build/bench/navtracker_cl4_a3_census --mode pending --scenario philos                 # r_chain 25
./build/bench/navtracker_cl4_a3_census --mode pending --scenario philos --chain-radius 15
for S in 13 16 17 22; do
  ./build/bench/navtracker_cl4_a3_census --mode pending --scenario autoferry_scenario$S
done
```

## Decision (the user's)

The endgame choice, unchanged in ownership: (a) one-shot `W25/f0.10` integration
(cheapest lever, 25–50 m reach), (b) a pending-band Phase-2 build (full 0–50 m
reach, ~7–17 s latency, matches the front's phantom cost at K≈5–8), or (c) keep
today's default. This probe closes the shape question (Step 1: shape is a red
herring — any one-shot scheme is on the swept surface) and prices the pending
band (Steps 2–3: matches the front, does not beat it, small persistent-offshore
residual). **No shipping default recommended here.**
