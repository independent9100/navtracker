# 2026-07-08 — backlog #25 Phase 1: what kills the PMBM track at close passes (localization)

**Phase 1 localization only — no fix, no config change, no lever.** This pass
answers the single question backlog #25 asks: on the sustained close crossings
where PMBM (`imm_cv_ct_pmbm_coverage_land`) drops a target for tens of seconds
at the CPA (Imazu Q2b, `docs/baselines/2026-07-08_imazu22.md`), **which stage
kills the track** — existence miss-starvation (H1), abrupt structural pruning
(H2), or estimator state divergence (H3)? The Phase-2 design happens at the
arbiter after this report. North-star tag: **Cl-3** (PMBM = academic-SOTA arm;
this is ADR 0002's forbidden failure in temporal form — a persistent real
object represented as *nothing* during the CPA window).

**Headline verdict: H3 (estimator state divergence) is the killer; H1
(miss-starvation) is only a downstream secondary effect; H2 (structural death)
is absent.** This *refutes* the arbiter's H1 working hypothesis: the target is
lost because the Bernoulli tracking it has its **velocity state run away**
(hundreds → thousands → unbounded m/s; true targets move ~4 m/s), so the
estimate leaves the 100 m match gate **while its existence `r` is still high**
(0.99–1.0 at the instant it crosses out). The truth is unrepresented because its
confirmed track *flew off*, not because its existence starved in place. Existence
`r` decay, where it happens, is a *consequence* of the divergence, not the cause
(imazu_22 id 7: r stays pinned 0.98–1.0 the whole 96 s; imazu_15 id 6: r decays
1.0 → ~0.5 only *after* it is already kilometres off the target, and never
reaches the 0.1 output floor in-window). That changes the Phase-2 lever class
entirely — estimator/association robustness, not existence floors / birth.

**Adversarially verified (independent re-analysis of the raw diag CSVs):** the
causal claim (H3 primary, H2 absent, H1 not-primary) holds; the re-analysis
sharpened two things folded into the headline below — (a) velocity runaway is a
*systemic* IMM filter-stability defect, not CPA-unique (see Controls), and (b) a
**neighbor-lock / measurement-hijack** data-association effect prolongs the loss
and delays re-acquisition (see Mechanism). Both are incorporated below.

## The three hypotheses (prior order) and their signatures

- **H1 — miss-starvation** (the arbiter's working hypothesis): the neighbour
  track claims the contested measurements scan after scan; the losing track
  takes repeated misses; its existence `r` decays *smoothly below the
  deletion/output threshold* while in-gate measurements exist. Signature:
  gradual `r` decay to the floor, `claimed_meas = −1`, track stays *near* the
  truth.
- **H2 — abrupt structural death**: hypothesis-cap pruning / Murty-K limit /
  r_min recycling drops the Bernoulli in one step. Signature: `r` healthy on
  scan k, gone on k+1; a `hyp_dropped_floor` / `hyp_dropped_cap` /
  `bernoulli_pruned_rmin` event coincides with a healthy→gone transition.
- **H3 — estimator divergence first**: the state walks off during the
  ambiguity; the gate follows it away from the true return; misses follow
  because the gate is in the wrong place. Signature: state/velocity blow-up
  and distance-from-truth blow-up *while `r` stays high*.

## What was instrumented (additive, default-off, byte-identical hook)

The existing `--export-states-dir` output is position-only
(`scan,time_s,kind,id,east_m,north_m`) and cannot see any of the H1/H2/H3
discriminators — `PmbmTracker::tracks()` collapses the MBM to Confirmed tracks
and discards `r`, association, and structural events. Phase 1 therefore adds a
**diagnostic-only** per-scan PMBM introspection hook:

- `core/pmbm/PmbmDiagnostics.hpp` — `IPmbmDiagnosticSink` + records. Lives
  beside the tracker (not in `ports/`): it is forensics, not a consumer
  integration port.
- `PmbmTracker::setDiagnosticSink(...)` — a single nullable per-instance
  pointer (no globals — the no-global-toggles rule). Every diagnostic
  computation is guarded on it; **null (default) = zero overhead, byte-identical
  tracking**. Emitted once per `processBatch`, after `pruneAndNormalise`, so it
  observes the exact post-scan density the states snapshot reads. It never
  mutates tracking state.
- `core/benchmark/PmbmDiagRecorder.hpp` + bench flag `--export-pmbm-diag-dir`
  (threaded through `SweepParams`) — streams two CSVs per PMBM run:
  - `<..>.pmbmbern.csv`: per scan per aggregated identity —
    `scan,time_s,id,agg_mass,r_best,hyp_count,claimed_meas,east_m,north_m,speed_mps,in_dominant,in_output,confirmed`.
    `agg_mass = Σ_j w^j·r^{j,id}` is the exact existence mass the output floors
    at `output_existence_floor=0.1` (so a track vanishes from the states export
    precisely when this crosses 0.1). `speed_mps = √(vx²+vy²)` of the dominant
    Bernoulli state — the divergence signal. `claimed_meas` = the measurement
    the dominant Bernoulli claimed (−1 misdetected).
  - `<..>.pmbmscan.csv`: per scan — `n_meas,n_hyp,n_bernoulli,n_ids,`
    `hyp_dropped_floor,hyp_dropped_cap,bernoulli_pruned_rmin` (the structural
    events that would signal H2).

NIS-per-update is **not** emitted: PMBM has no innovation sink, and emitting it
inside the per-Bernoulli×per-measurement×per-hypothesis update (most of which is
pruned) is both invasive and ambiguous. The **state-divergence signal** (dominant
Bernoulli position + `speed_mps`, read at output time) is the direct and cleaner
H3 witness and needs no update-path surgery. This is the one deliberate
deviation from the ticket's instrument list, made to keep the hook minimal.

### Byte-identical proof (acceptance #3)

The binary that produced the `states.csv` used below was built *before* the hook
edits; re-running the *post-hook* binary — **with the diagnostic sink active** —
reproduces every `states.csv` bit-for-bit:

- **All 22 Imazu `states.csv` byte-identical** pre-hook vs post-hook-with-diag
  (e.g. `imazu_15` sha256 `da66533e…` identical both sides).
- **Non-imazu (the ticket's explicit ask):** `imm_cv_ct_pmbm_coverage_land` on
  all **32** `--with-simms` scenarios, diag-off vs diag-on: all 32 `states.csv`
  byte-identical; **1488/1488 accuracy-metric rows byte-identical**. The *only*
  differing metrics are the four `scan_proc_ms_*` per-scan **wall-clock latency**
  values — non-deterministic run-to-run regardless, and diag-on legitimately
  adds its own emission cost. No tracking-accuracy metric moves.

## Method and its faithfulness anchor

`tools/pmbm_closepass_trace.py` consumes the position export + the two new diag
CSVs + the fixture `ownship.csv`. It reuses the *proven-faithful* per-scan
Hungarian assignment and loss-window logic from the #11 tools
(`imazu_switch_forensics.load_states/assign_all`, `imazu_trackloss.loss_windows`
— whose per-truth `id_switches`/`track_breaks` equal the bench's own metric rows
on all 22 cases). The states `scan` index is the 1 Hz truth tick (scan ==
second); the diag rows are per `processBatch` (radar every 2.5 s + AIS at
fractional times), so they are joined to the truth timeline by an **as-of** join
(the last diag row with `time ≤` the truth-scan time — exactly the density the
snapshot reads).

**Faithfulness is proven, not assumed.** Every Confirmed track in the states
export is contained in the diag with a matching position:

| case | states confirmed tracks contained in diag (mass≥0.5, pos match) | max pos err |
|---|---|---|
| imazu_15 | 2155 / 2155 (100.00%) | 0.000 m |
| imazu_22 | 2085 / 2085 (100.00%) | 0.000 m |
| imazu_12 | 2656 / 2656 (100.00%) | 0.000 m |
| imazu_08 | 2055 / 2055 (100.00%) | 0.000 m |
| imazu_01 | 1273 / 1273 (100.00%) | 0.000 m |

The diag deliberately surfaces *more* than the states export — sub-floor
Bernoullis (the point of the hook) and Coasting-status tracks, which
`snapshotAtPmbm` drops via its `status==Confirmed` filter while
`refreshAggregatedTracks` marks cooperative-overdue ids Coasting
(`PmbmTracker.cpp:2226`) even though their mass stays ≥ 0.5. So the anchor is
containment (states ⊆ diag) + exact position agreement, which holds at 100%.

## The verdict, with the trace evidence

### imazu_15, truth 257010151 — the 158 s loss (id 6→504, overlaps own-ship CPA @474)

Confirmed-track pool collapses from 3 to 2 before the loss; the target's
confirmed track (id 6) then **diverges**: `r`/`agg_mass` stays high the whole
time while `speed_mps` and distance-from-truth blow up.

| scan | truth assigned? | id 6: agg_mass | r_best | claimed | speed (m/s) | dist to truth (m) | struct floor/cap/rmin |
|---|---|---|---|---|---|---|---|
| 475 | – | 1.00 | 1.00 |  0 | 133 | 246 | 0/0/0 |
| 476 | **6** | 1.00 | 1.00 |  0 | **792** | 51 | 0/0/0 |
| 477 | **6** | 1.00 | 1.00 |  0 | 839 | 55 | 0/0/0 |
| 478 | – | 1.00 | 1.00 |  0 | 839 | **1838** | 0/0/0 |
| 480 | – | 0.99 | 0.99 | −1 | 732 | 3 653 | 0/0/0 |
| 492 | – | 0.96 | 0.96 | −1 | 885 | 10 205 | 0/0/0 |
| 507 | – | 0.85 | 0.85 | −1 | 8 050 | 23 844 | 0/0/0 |
| 515 | – | 0.81 | 0.81 | −1 | 124 442 | 155 407 | 0/0/0 |

id 6 briefly re-grabs the target at scan 476 (dist 51 m) — and that large
position correction is absorbed into velocity (speed jumps 133 → 792 m/s). From
478 on it coasts on that corrupted velocity and flies off (→ 155 km by scan
515). Its existence **never starves in-window**: `r` stays 1.0 through the moment
it leaves the 100 m match gate (scan 478, dist 1,838 m, r=1.0) and only *coasts*
down to **0.81 by scan 515** — still Confirmed, 155 km off target — decaying by
the mild per-scan `(1−p_D)` factor once it stops being fed, never approaching the
0.1 output floor. The target is lost purely on **position** (the estimate left
the gate), not on existence. **No structural events** (floor/cap = 0 throughout).
This is H3, not H1. (id 6 leaves the Confirmed *states* output around scan 496 —
a metric/status effect — but the diag shows the Bernoulli itself is alive and
diverging far beyond that; that persistence is exactly what the states export
cannot see.)

### imazu_22, truth 257010221 — the 96 s loss (id 7→472, overlaps own-ship CPA)

Even sharper: the confirmed pool collapses to **one** track (nConf = 1) serving
three truths, and that track (id 7) has **r pinned at 0.98–1.0 for the entire
96 s** while its velocity sits at **3,700–4,500 m/s** and it thrashes between
targets (grabbing a measurement, dist → 31 m; then flung off, dist → 1,465 m;
repeat). The *same* id 7 is the "dying track" for both truth 221 and truth 222 —
one diverged track nominally straddling two targets (coalescence).

| scan | truth assigned? | id 7: agg_mass | r_best | claimed | speed (m/s) | dist to truth (m) | struct floor/cap/rmin |
|---|---|---|---|---|---|---|---|
| 490 | – | 1.00 | 1.00 | 3 | 3 836 | 228 | 0/0/0 |
| 495 | – | 1.00 | 1.00 | 4 | 3 774 | 244 | 0/0/0 |
| 498 | **7** | 1.00 | 1.00 | 0 | 4 456 | 31 | 0/0/0 |
| 500 | – | 1.00 | 1.00 | 8 | 3 928 | 247 | 0/0/0 |
| 505 | – | 1.00 | 1.00 | 0 | 297 | 416 | 0/0/0 |
| 513 | – | 0.99 | 0.99 | −1 | 184 | 543 | 0/0/1 |

r never drops below 0.98 across the loss; the target is unassigned because the
track is 200–1,500 m away with an unphysical velocity, not because it starved.

### All six dying tracks (two worst cases + their neighbours)

| case | truth | dying id | mechanism | signature at loss onset |
|---|---|---|---|---|
| imazu_15 | 257010151 | 6 | **H3** | r 1.0, speed 792→unbounded, dist 51→156 km |
| imazu_15 | 257010152 | 7 | **H3** | r high, 50/50 loss scans speed 117–348, dist >200 m |
| imazu_15 | 257010153 | 517 | **H1+H2+H3** | re-birth diverges (3 scans) *then* mass 0.91→0.01 over 51 scans (tail starvation, permanent) |
| imazu_22 | 257010221 | 7 | **H3** | r 0.98–1.0, speed 3,928, thrash |
| imazu_22 | 257010222 | 7 | **H3** | same id 7 covers 2 truths (coalescence) |
| imazu_22 | 257010223 | 1 | **H3** | r high, speed 980→unbounded |

Five of six are pure H3. The one mixture (imazu_15 truth 153) is a *re-acquired*
track that diverges mildly and *then* miss-starves to a permanent loss at
run-end — i.e. H1 appears as the **downstream cleanup of a target already
orphaned by divergence**, not as the initial kill at the CPA.

### Controls (the mechanism scales with ambiguity duration)

| case | pass | confirmed track through pass | speed | verdict |
|---|---|---|---|---|
| imazu_01 | single target | **primary** track holds (dist < 30 m, id 8 ~2–3 m/s) | see caveat | clean (no substantial loss) |
| imazu_08 | 85 m | holds (dist < 70 m), 4 s flicker | 200–450 m/s (mild) | clean (no substantial loss) |
| imazu_12 | 0.6 m **fleeting** | identity held through the pass | id 1 diverges to 25 km / 1.2e5 m/s over 45 s **before** the CPA | H3 present but pre-CPA |

The `speed_mps` column is meaningful (the *well-tracked* population — confirmed
tracks within 100 m of a truth — has median speed ~6 m/s; imazu_01's primary
track id 8 sits at ~2–3 m/s). **But velocity runaway is SYSTEMIC, not
CPA-unique:** even single-target imazu_01 carries diverged-but-still-Confirmed
tracks — **439 of 959 confirmed-track rows (46 %) have speed > 50 m/s**, median
17 m/s, tail to 1e16 m/s. These are the same phantom/duplicate population as the
crossing-independent +0.77 clutter over-count (17–49 live Bernoulli ids for 3
truths). So the IMM CV/CT filter has *no* velocity/innovation bound in general.
What the *sustained close pass* adds is narrower and decisive: it (a) corrupts
the **target's own** track velocity via mis-association and (b) deprives that
track of its measurement stream, so an already-inflated velocity carries it off
the target unbounded — overlapping the CPA. In the clean/mild cases the target's
primary track keeps detecting every scan and stays inside the gate despite any
inflation. This matches Q2b's "sustained proximity, not closeness per se".

## Why H2 is ruled out and H1 is only secondary

Across every loss window analysed, `hyp_dropped_floor = 0` and
`hyp_dropped_cap = 0` — the mixture runs as a **single global hypothesis**
(K=1 GNN), so there is no hypothesis-cap or weight-floor pressure to drop a
Bernoulli abruptly. `bernoulli_pruned_rmin` only ever removes *already-sub-floor
phantom* Bernoullis (the crossing-independent clutter over-count, 17–49 live
ids for 3 truths), never the tracked target in a healthy→gone step. The single
H2 flag (imazu_15 truth 153) is a mass cliff at the *tail* of an
already-H1-decaying re-acquired track, not a healthy-track cliff. H1's smooth
`r`-decay-to-floor signature appears only on that same tail case — never on the
primary track at the CPA, whose `r` stays ≈ 1.0.

## The mechanism, stated precisely (for the Phase-2 design)

Under sustained close proximity the wide validation gate (`gate_threshold` = 20,
≈ χ²₂ 99.99%) admits both targets' returns into a track's region; the **K=1
winner-take-all** assignment hard-commits the track to one measurement per scan.
When the track has drifted during the ambiguity and then re-grabs the true (or a
neighbour's) return, the large position innovation is absorbed partly into the
**velocity** state (high covariance → high Kalman gain), overshooting; the
IMM CV/CT filter has no speed/innovation bound, so the velocity grows, the
predicted position leaps further each scan, the position covariance and gate
widen, ever more distant returns are admitted, and the estimate runs away —
classic mis-association-driven filter divergence. Existence `r` stays high
at the instant the estimate leaves the gate; thereafter, if the diverging track
keeps claiming *some* measurement (clutter or a neighbour) `r` stays pinned
(imazu_22 id 7: 0.98–1.0), and if it coasts unfed it decays only by the mild
per-scan `(1−p_D)` factor (imazu_15 id 6: 1.0 → ~0.5 over ~100 s, never reaching
the 0.1 floor in-window). Either way the abandoned target has no confirmed track
within the 100 m metric gate → the Q2b "loss".

**Why the loss lasts 96–158 s and re-acquires late — a neighbour-lock /
measurement-hijack co-mechanism (data association, concurrent with H3).** Two
things keep the target uncovered after its track diverges: (a) the surviving
confirmed tracks are *locked onto the neighbour truths* (in imazu_15, after id 6
flies off, id 7 sits 5–40 m from truths 152/153 but 280–670 m from the abandoned
151), and (b) the diverged track keeps *hijacking* the abandoned target's returns
(K=1 winner-take-all lets id 7 in imazu_22 claim a measurement on alternate
scans, holding r = 1.0), which starves a fresh birth on the abandoned target.
The re-birth only succeeds once the diverged track finally stops claiming and the
geometry opens — hence the delayed re-acquisition under a new id. This is a
distinct association failure the verdict counts alongside H3, not a separate
existence-decay (H1) story.

**Systemic vs CPA-specific (verified).** The velocity runaway itself is a general
IMM filter-stability defect — diverged-but-Confirmed tracks exist even in the
single-target control (imazu_01: 46 % of confirmed-track rows > 50 m/s). The
sustained close pass is what steers the divergence onto the *target's own* track
and strips its measurement stream. So a Phase-2 velocity/innovation stability
bound would help globally (and likely also dents the crossing-independent clutter
over-count), not only at the CPA.

## Phase-2 conditioning note (one paragraph, no build)

The suspected-H1 miss penalty is the SAME brake that suppresses philos phantom
over-count: the misdetection existence recursion (and, on the legacy path, its
`(1−p_D)^N` over-penalty) is what decays shore/structure phantoms
(`docs/algorithms/pmbm-design.md §3.1.1`, the 2026-06-24 cardinality line). The
good news from this localization is that the killer is **not** on that channel
at all — it is estimator-side (velocity runaway under hard mis-association), so
the natural Phase-2 levers (a maritime speed / innovation gate that rejects
updates implying non-physical velocity; an ambiguity-aware *soft* update instead
of hard winner-take-all during contested scans — note the PDA soft-detected
branch already exists but is OFF; or an explicit coalescence/divergence guard)
touch the **update/gating** path, not the misdetection existence recursion, and
therefore cannot weaken the philos over-count brake by construction. The
condition a fix must respect is the *inverse* of #11's: it must fire **only in
the association-ambiguity / large-innovation context** (a confirmed track being
pulled by a contested or oversized-innovation update at a close pass) and must
**leave the existence/birth channel untouched** — so it neither re-opens the
crossing-independent +0.77 clutter over-count nor the parked λ_C-birth invariant,
and it must be per-instance / opt-in / default-byte-identical and re-measured on
the philos + HAXR KEEP configs before promotion. (Caveat for the PDA-soft
candidate specifically: it previously regressed open-sea lifetime when applied
globally, so it must be gated to contested close-pass scans, not switched on
wholesale.)

## Reproduce

Bench + diagnostic export are byte-deterministic (re-run checksum verified). All
numbers above are seed 0. `SIMMS_DIR` → the main-tree fixtures (gitignored;
present only in the main tree).

```
# 1. Export positions + the new PMBM diagnostics for the Imazu battery:
SIMMS_DIR=$PWD/tests/fixtures/sim_multisensor ./build/bench/navtracker_bench_baseline \
  --with-imazu --skip-replays --scenario-filter imazu --seeds 1 \
  --config-eq imm_cv_ct_pmbm_coverage_land --run-id b25_pmbm \
  --out <dir> --export-states-dir <dir>/states_pmbm \
  --export-pmbm-diag-dir <dir>/pmbm_diag

# 2. Scan-by-scan trace + faithfulness anchor + mechanism verdict (per case):
tools/pmbm_closepass_trace.py \
  --states  <dir>/states_pmbm/imm_cv_ct_pmbm_coverage_land__imazu_15__seed0.states.csv \
  --diag-bern <dir>/pmbm_diag/imm_cv_ct_pmbm_coverage_land__imazu_15__seed0.pmbmbern.csv \
  --diag-scan <dir>/pmbm_diag/imm_cv_ct_pmbm_coverage_land__imazu_15__seed0.pmbmscan.csv \
  --ownship tests/fixtures/sim_multisensor/imazu_15_s0/ownship.csv --label imazu_15
  # …repeat --scenario imazu_22 (dying), imazu_12/08/01 (controls); --truth <id> for a specific target

# 3. Byte-identical proof (non-imazu; diag off vs on → identical states + accuracy metrics):
SIMMS_DIR=$PWD/tests/fixtures/sim_multisensor ./build/bench/navtracker_bench_baseline \
  --with-simms --skip-replays --config-eq imm_cv_ct_pmbm_coverage_land --seeds 1 \
  --run-id A --out <A> --export-states-dir <A>/st
#  …repeat as run B adding --export-pmbm-diag-dir <B>/diag; diff <A>/st vs <B>/st (identical),
#  and diff the metric CSVs excluding wall_seconds + scan_proc_ms (identical).
```

Input fixtures are unchanged — their sha256 prefixes are the 2026-07-08 set in
`docs/algorithms/evaluation-log.md`. New reproducer: `tools/pmbm_closepass_trace.py`
sha256 `7f0ba18c`. The exported states CSVs are byte-identical to the pre-hook
2026-07-08 Imazu set (e.g. `imazu_15` states `da66533e…`).

## Acceptance / stop-and-report

- **Loss reproduces exactly** (determinism confirmed): `imazu_trackloss.py` on
  the fresh export reproduces the Q2b table to the count — imazu_15 = 61 windows
  / 657 s / 18 new-id / 2 permanent / longest 158 s; imazu_22 = 97 / 709 / 27 /
  1 / 96 s. No determinism alarm.
- **Zero behaviour change:** additive, default-off, per-instance hook; byte-
  identical proof above.
- The needed quantities *could* be exposed without touching tracking behaviour
  (the additive hook), so acceptance #5 resolves negative-for-stop.

## Files added / changed

- `core/pmbm/PmbmDiagnostics.hpp` (new) — `IPmbmDiagnosticSink` + records.
- `core/benchmark/PmbmDiagRecorder.hpp` (new) — CSV recorder.
- `tools/pmbm_closepass_trace.py` (new) — reproducer.
- `core/pmbm/PmbmTracker.{hpp,cpp}` — nullable diag sink + guarded per-scan
  emission + guarded structural-event counters (byte-identical when null).
- `core/benchmark/Sweep.{hpp,cpp}`, `bench/baseline_matrix.cpp` — the
  `--export-pmbm-diag-dir` flag plumbed to the recorder wiring.
