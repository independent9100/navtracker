# Backlog #25 Phase 2a — runaway census + offline velocity/innovation-bound probe

*Executed 2026-07-08 (Cl-3, #25 deployment-choice discriminator). No build of any
lever; zero core/config change. Branch `backlog25-phase2a`.*

## Why this phase exists

Phase 1 (`docs/baselines/2026-07-08_b25_localization.md`, merged `3ae355f`)
localized the PMBM close-pass track loss to **H3 — estimator state divergence**:
at a sustained close pass the Bernoulli's velocity state runs away, the estimate
leaves the 100 m gate while existence `r` is still high, and the truth is left
unassigned across the own-ship CPA. Phase 1 also flagged a *systemic surprise*:
diverged-but-still-Confirmed tracks exist even in single-target `imazu_01`
(46 % of confirmed rows > 50 m/s).

Before **building** any lever, this phase answers two questions offline, with
binding kill-criteria — the clutter-campaign discipline (a §5.0-style probe kills
a doomed build before it is written). All numbers come from the **existing**
Phase-1 diagnostic export (`IPmbmDiagnosticSink` + `--export-pmbm-diag-dir`) fed
to a new read-only analysis script `tools/pmbm_phase2a_probe.py`. Nothing in the
tracker changed; the bench binary is byte-for-byte master's (determinism
re-verified below).

The canonical config is `imm_cv_ct_pmbm_coverage_land` (the #25 subject),
seed 0. Datasets: 22 Imazu + 6 sim_ms (census + probe) and 18 AutoFerry
(9 unanchored "real workload" + 9 anchored) for the false-fire side.

---

## Question A — the runaway census (who diverges, really?)

Over every **Confirmed** diag row with implied speed > 50 m/s (across 22 Imazu +
6 sim_ms), attributing each row to its track's **birth class** (target-born =
first appears ≤ 100 m from a truth; else clutter-born), whether the scan is
**contested** (≥ 2 Confirmed tracks within 200 m), and the **update cadence**
(seconds since the track's last *detected* update):

| metric | value |
|---|---|
| confirmed rows with speed > 50 m/s | **19 869** |
| target-born-track rows | 3 470 (**17.5 %**) — 46 distinct tracks |
| **clutter-born-track rows** | 16 399 (**82.5 %**) — 930 distinct tracks |
| contested (≥2 conf tracks < 200 m) | 603 (3.0 %) |
| median lifetime, target-born tracks | **717.5 s** (whole scenario) |
| median lifetime, clutter-born tracks | **117.5 s** (short-lived) |
| median update-gap at runaway rows | **17.5 s** (they are *coasting*, not being fed) |
| (c) dominant IMM mode | **not exported** — see Stop-and-report |

**Single-target scenarios** (`imazu_01–04`, exactly one truth ever present):
1 852 runaway rows, of which **1 842 (99.5 %) are clutter-born** and only 10
(0.5 %) target-born.

Per-scenario (runaway rows; target/clutter rows; target/clutter distinct tracks;
contested scans):

```
scen                nTruth  runaway  tgt/clt rows   tgt/clt tracks  contested
imazu_01 (single)     1       439      0 / 439        0 / 34            0
imazu_02 (single)     1       487      0 / 487        0 / 36            0
imazu_03 (single)     1       479      3 / 476        1 / 36            0
imazu_04 (single)     1       447      7 / 440        1 / 35            0
imazu_05              2       920    149 / 771        2 / 44            0
imazu_08              2      1069    204 / 865        2 / 42            1
imazu_14              3       925    414 / 511        2 / 26           92
imazu_15              3      1052    394 / 658        2 / 39           72
imazu_17              3       722    242 / 480        3 / 26           43
imazu_20              3       802    230 / 572        2 / 35           67
imazu_21              3       974    128 / 846        3 / 32          137
imazu_22              3       846    262 / 584        2 / 28           13
sim_ms_clutter_burst  2       733      1 / 732        1 / 57           42
sim_ms_crossing       3       340    164 / 176        2 / 11            0
sim_ms_headon         2       337    125 / 212        2 / 19            0
   (…full 28-row table in the raw probe output; representative rows shown)
```

### What the census says (explicitly, either way)

**Phase-1's "systemic IMM defect" reading shrinks.** In single-target
scenarios 99.5 % of the runaway rows belong to **short-lived clutter-born
Bernoulli phantoms** (median life 117 s), not to the target's own track. The
"46 % of `imazu_01` confirmed rows > 50 m/s" is dominated by dozens of clutter
phantoms that confirm briefly and blow up — **not** the primary target track
losing its mind. The target track (`imazu_01` id 8) is long-lived and, except
for brief spikes, well-tracked. So a velocity/position bound would **double as a
clutter-phantom killer** — the 82.5 % clutter-born majority is exactly the
population it would suppress.

**But the CPA-dying tracks are genuinely target-born.** The Phase-1 dying
tracks are *not* phantoms — they are long-lived target-born tracks
(`imazu_15` id 6: born 47 m from truth, 595 s life; id 7: 74 m, 718 s;
`imazu_22` id 7: 718 s) that diverge specifically at the close pass. Two of the
six dying (truth, id) rows are the coalescence/re-birth cases (`imazu_15` id 517
is a clutter re-birth at t=608; `imazu_22` id 1 is born borderline at 117 m).
So **H3-at-the-CPA is a real target-track failure** distinct from the phantom
population — Phase 1's mechanism stands; only its "systemic across all rows"
gloss is reweighted toward clutter.

**Contest is rare in aggregate (3 %) but concentrated at the dense CPAs**
(`imazu_21` 137, `imazu_14/15/20` 67–92). Most runaway rows are *uncontested
coasting phantoms*; the *target-track* divergences, though, sit in the contested
close-pass geometry — consistent with mis-association triggering the target
divergence while phantoms diverge on their own.

---

## Question B — offline probe of a velocity/innovation bound

The lever is simulated **on paper** from the traces: a track update is *flagged*
when it implies **speed > V_max** (from `speed_mps` directly) **or** a
**position displacement > D_max** metres. V_max ∈ {25, 50, 75} m/s (≈ 50–150 kn),
D_max ∈ {100, 200, 400} m.

> **Export limitation (declared).** The true *measurement innovation*
> (‖z − H x_pred‖) is **not** in the diag export. The D_max axis is therefore a
> **proxy**: the per-diag-row posterior position jump ‖pos(k) − pos(k−1)‖. This
> is the observable consequence of a large correction/coast and is checkable at
> the estimator predict/update step, but it is **not** identical to an
> association-gate innovation. See Stop-and-report + the Phase-2b note.

### B.1 Detection side — do the six dying tracks get flagged before the loss?

"Gate-exit" is reported two ways because the dying tracks **thrash** (leave the
100 m gate, re-acquire the target, leave again) before a *permanent* departure:
`first-excursion` = onset of trouble; `PERMANENT gate-exit` = the point of no
return (dist grows monotonically thereafter). The binding reference is the
permanent exit. The loss overlaps the own-ship **CPA** in every case.

| dying track | first-exc t | PERMANENT exit t | CPA scan (dist) | flagged before permanent exit? |
|---|---|---|---|---|
| imazu_15 truth …151 id 6 | 402.9 | 477.5 | 474 (6 m) | **yes** (V or D) |
| imazu_15 truth …152 id 7 | 362.5 | 697.5 | 441 (14 m) | **yes** |
| imazu_15 truth …153 id 517 | 670.0 | 670.0 | 451 (5 m) | **yes** (+17.5 s only) |
| imazu_22 truth …221 id 7 | 360.0 | 660.0 | 474 (6 m) | **yes** |
| imazu_22 truth …222 id 7 | 382.9 | *none* | 510 (11 m) | n/a (coalesced onto shared id 7, no clean exit) |
| imazu_22 truth …223 id 1 | 345.0 | 432.5 | 450 (14 m) | **yes** |

**5 of 6** dying tracks are flagged before their permanent gate-exit, for
**every** (V_max, D_max) — meeting the ≥ 5/6 detection bar. The 6th
(`imazu_22` truth 222) never gets a clean permanent exit because id 7 **coalesces
onto two truths at once** (Phase-1's coalescence case) — it is not a miss of the
bound.

> **Margin caveat (honest).** At the close pass the dying ids **coalesce/swap
> across neighbouring truths**: e.g. `imazu_15` id 6 tracks truth 152/153 at low
> speed until ~t=350 (id 7 holds truth 151), then migrates onto truth 151 at the
> CPA and diverges. So the *per-truth flag-time margins* (nominally +17 to +615 s
> before permanent exit) are **confounded** by this coalescence and are **not**
> clean per-truth lead-times. The robust statement is the binary one: the
> diverging Bernoulli **is** flagged before truth-151 is permanently lost, and
> before the CPA.

### B.2 False-fire side — how often does the bound flag a HEALTHY track?

Healthy = a Confirmed track that **is the Hungarian-assigned tracker of a truth**
at that scan (excludes fast phantoms merely wandering within 100 m of a truth —
those we *want* to flag). Fraction of healthy confirmed track-rows flagged, per
axis, per dataset:

| dataset (healthy rows) | SPEED>25 | SPEED>50 | SPEED>75 | JUMP>100 | JUMP>200 | JUMP>400 |
|---|---|---|---|---|---|---|
| **autoferry_unanch** (10 322) — REAL | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % |
| autoferry_anchored (35 445) | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % |
| **sim_ms** (5 417) | 11.39 % | 8.18 % | 6.61 % | 1.13 % | **0.13 %** | **0.00 %** |
| imazu single-target (1 384) | 4.41 % | 2.60 % | 2.17 % | 0.29 % | 0.00 % | 0.00 % |

**Why SPEED false-fires and JUMP does not.** The sim_ms healthy tracks that trip
the speed bound are **well-positioned** — median 22–31 m from truth (min 1.2 m) —
yet carry a *transient velocity-state spike* of 159–235 m/s during crossing/
head-on association ambiguity. A measurement snaps their **position** back to the
target while the IMM **velocity state** momentarily overshoots. So implied speed
is polluted by a benign artifact, whereas the **actual position displacement**
stays small for a corrected healthy track and only blows up for a genuine
divergence. This is the whole discriminator.

### B.3 Binding kill-criteria

> Graduate to a Phase-2b build **only if** some swept setting flags ≥ 5/6 dying
> tracks before gate-exit **and** false-fires on < 1 % of healthy confirmed
> track-scans on **autoferry_unanchored + sim_ms**.

| lever axis / setting | dying before exit | false-fire (af_unanch + sim_ms) | verdict |
|---|---|---|---|
| SPEED (V=25) | 5/6 | 3.92 % | ✗ |
| SPEED (V=50) | 5/6 | 2.82 % | ✗ |
| SPEED (V=75) | 5/6 | 2.28 % | ✗ |
| **JUMP (D=100)** | 5/6 | 0.39 % | ✓ |
| **JUMP (D=200)** | 5/6 | **0.044 %** | ✓ |
| **JUMP (D=400)** | 5/6 | **0.000 %** | ✓ |
| OR(speed, jump) — any combo | 5/6 | 1.9–3.9 % | ✗ (speed drags it) |

The **velocity bound fails**; the **position-displacement (innovation-proxy)
bound passes** at D_max ≥ 100 m, cleanly at D_max ≥ 200 m.

---

## Verdict (one page)

**BUILD — but the position-innovation gate, NOT the velocity bound.** The
originally-hypothesised velocity/implied-speed bound is a **NO-BUILD**: it fails
the binding false-fire criterion (2–11 % on sim_ms + imazu single-target),
because a healthy well-positioned track transiently carries a huge velocity
*state* during close-pass ambiguity. The **position-displacement component**
(D_max) meets both binding criteria — 5/6 dying tracks flagged before permanent
gate-exit and ≤ 0.13 % false-fire on every honest-truth dataset at D_max ≥ 200 m
— because it keys on where the track actually *is*, not what its velocity state
transiently claims.

**Recommended starting band for the Phase-2b ticket:** D_max ≈ **200–400 m**
per update step (0.13 %→0.00 % false-fire, still 5/6 detection). Do **not** ship
a speed/`speed_mps` threshold as the trigger.

**Where the guard belongs:** the discriminator is *position moved per update*,
which is available at the **estimator predict/update step** (a per-track
displacement / velocity×Δt sanity check) and, in its true form, at the
**association gate** (the measurement-innovation). The guard should **clamp or
reject the corrupting kinematic update / association — never delete the
Bernoulli** (existence stays untouched; see conditioning note).

**Two things Phase-2b must resolve before committing the build:**

1. **Confirm on the *true* innovation.** The offline D-axis is a posterior
   position-jump proxy; it cannot be distinguished offline from an
   association-gate innovation gate. Add one additive field (measurement −
   predicted position, ‖·‖) to `IPmbmDiagnosticSink` — default-off,
   byte-identical, the same pattern as Phase 1 — and re-run this probe to pick
   between an estimator-side displacement clamp and an association-side
   innovation gate.
2. **Pair it with a coalescence guard.** The per-truth margins are confounded by
   close-pass coalescence (dying ids migrate across near truths; `imazu_22` id 7
   covers two truths). The divergence is *entangled* with tracks merging, so a
   displacement gate alone treats a symptom.

**Ranked alternates** (per the campaign rule):

1. **Position-innovation / displacement gate** — passes the offline probe;
   top candidate, gated on the Phase-2b innovation-export confirmation above.
2. **Coalescence guard** — directly addresses the entanglement the census +
   detection trace exposed (cross-truth id migration at the CPA).
3. **Ambiguity-gated soft update** — the PDA soft detected-branch
   (`imm_cv_ct_pmbm_land_pda`) exists but is OFF because it regressed open-sea
   lifetime globally when always-on; restrict it to *contested close-pass scans*
   (the 3 %/dense-CPA subset the census isolates) so it cannot hurt the
   open-sea majority.

### Phase-2b conditioning note (must not weaken the philos over-count brake)

The philos over-count "brake" is the **miss-P_D existence over-penalty** — it
lives entirely in the **existence recursion** (`pmbm-design.md §3.1.1`). The
recommended lever acts on the **kinematic/association path** (velocity state,
position displacement, innovation) and must **not touch existence or birth**. So
by construction it cannot weaken the brake — *provided* it clamps/rejects
kinematics rather than deleting Bernoullis. Bonus alignment: since 82.5 % of the
runaway population is clutter-born phantoms that also blow up in position, a
displacement gate would *suppress* them, **helping** the over-count rather than
fighting the brake. The one forbidden implementation is a gate that **deletes**
the diverging Bernoulli (an existence action) — that would both bypass the brake
and risk ADR-0002 "presence over classification". Clamp kinematics, keep the
object present.

---

## Stop-and-report

- **IMM dominant mode (Question A attribute c) is not in the export.** Answered
  (a)/(b)/(d) fully; (c) is a genuine export gap. Distinguishing CV-vs-CT
  dominance needs the IMM mode probability exported (one additive field). It is
  **not** load-bearing for the verdict (which rests on the target/clutter split
  and the detection/false-fire numbers), so this phase reports the gap rather
  than adding a field (zero-core-change constraint). Deferred to the same
  Phase-2b instrumentation step as the innovation field.
- **True measurement-innovation is not in the export** — the D_max axis is the
  posterior-position-jump proxy (declared above). This does not block the
  verdict: the proxy already separates healthy from diverging cleanly; it only
  defers the *guard-placement* decision (estimator vs association) to a one-field
  Phase-2b re-probe.
- **Phase-1 H3 attribution is NOT confounded into invalidity, but it is
  reweighted.** The census does *not* show the dying tracks to be clutter-born
  (they are target-born, long-lived) — H3-at-the-CPA stands. What it reframes is
  the "systemic" gloss: the bulk of *all* runaway rows are clutter phantoms, and
  the close-pass divergence is entangled with coalescence. #25's
  deployment-choice framing is unchanged.

---

## Reproduce

Bench source is **unchanged from master** (`afc47d2`); the diag hook was merged
in Phase 1 (`3ae355f`). Determinism re-verified: an independent re-run of
`imazu_15` produced byte-identical `states.csv`, `pmbmbern.csv`, `pmbmscan.csv`.

```
# 1. Generate positions + PMBM diagnostics (PMBM canonical, seed 0).
#    SIMMS_DIR -> the MAIN-tree fixtures (gitignored; present only there).
SIMMS_DIR=<main>/tests/fixtures/sim_multisensor \
  ./build/bench/navtracker_bench_baseline --with-imazu --skip-replays \
  --scenario-filter imazu --seeds 1 --config-eq imm_cv_ct_pmbm_coverage_land \
  --run-id p2a_i --out <exp>/matrix \
  --export-states-dir <exp>/states --export-pmbm-diag-dir <exp>/diag
#   …repeat: --with-simms --scenario-filter sim_ms   (6 sim_ms)
#   …repeat: --scenario-filter autoferry             (18 autoferry; data/autoferry symlinked)

# 2. Census (A) + probe (B) + binding verdict + axis separation:
tools/pmbm_phase2a_probe.py \
  --states-dir <exp>/states --diag-dir <exp>/diag \
  --simms-dir <main>/tests/fixtures/sim_multisensor
```

- New reproducer `tools/pmbm_phase2a_probe.py` sha256 `bb09e933…` (stdlib only;
  read-only; reuses `pmbm_closepass_trace.py` `7f0ba18c…`,
  `imazu_switch_forensics.py` `9504189b…`, `imazu_trackloss.py` `39f0d090…` —
  all unchanged).
- Knobs (in-script, documented): SPEED_CENSUS 50 m/s, BORN_GATE 100 m,
  CONTEST_R 200 m, GATE 100 m, V_MAX {25,50,75}, D_MAX {100,200,400}.
- Datasets: 22 Imazu + 6 sim_ms + 18 AutoFerry, seed 0, config
  `imm_cv_ct_pmbm_coverage_land`.
