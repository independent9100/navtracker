# Cl-4 Phase 1 — ADR-0001-A3 sensor-aware-suppression evidence probe

**Date:** 2026-07-11 · **Branch:** `cl4-phase1-a3-probe` · **North-star:** Cl-4 Phase 1
(one deployable PMBM config). **Measurement only — zero shipped behaviour change.**
Ticket: `docs/superpowers/plans/2026-07-11-cl4-phase1-a3-probe-ticket.md`.

## Verdict (one line)

**NO-BUILD as specified.** The revival side is real and strong on env-2 (the missed
targets have abundant clutter-free-sensor evidence and are 100 % in-band), but the
guard side **cannot be certified on this gauntlet**: the sensors that would revive
env-2 (lidar, camera) are **entirely absent from philos**, and the one camera fixture
that exists never points at philos shore clutter. Worse, on the *only* workload that
carries cameras (env-2), a **camera-typed exemption is measured to re-admit ~80 %
clutter** — the ADR's warned failure mode, now quantified. Lidar is a clean
discriminator on env-2 but is a Trondheim-only sensor (Boston/philos has none), so it
cannot be the *one-config* fix either. **Recommend pivot to ranked path (b), the
conditional coverage floor** (arbiter writes that probe).

Tooling (reproducible): `tools/cl4_a3_census.cpp` → `navtracker_cl4_a3_census`
(added to `bench/CMakeLists.txt`; research-only, not on any tracker path).

---

## Step 0 — sensor-stream inventory

Fixed gate for all evidence tests (chosen up front, = the tracker's own association
gate, `ReplayScenarioRun.cpp:321`): **R = 15 m** position, **0.15 rad** bearing,
**±0.5 s** time window. "In-band" = the coverage_land no-birth zone, tested with the
tracker's own `CoastlineModel::clutterPrior(pos) > 0` (i.e. within 50 m offshore of
the loaded coastline; `CoastlineGeometry.cpp` ramp, `W_off = 50 m`). Birth
suppression there is `r_new *= (1−c)` with `min_new_bernoulli_existence ==
birth_existence_target == 0.1`, so any `c>0` kills the birth (`Config.cpp:1214-1223`).

| workload | targets | streams that reach the tracker | births vs update |
|---|---|---|---|
| **env-2** (autoferry sc13/16/17/22, Trondheim urban channel) | **2 moving** per scenario | **lidar** (Position2D, σ3 m, `autoferry_lidar`), **radar/ArpaTtm** (Position2D, `autoferry_radar`), **EO** + **IR** cameras (Bearing2D, `autoferry_eo`/`autoferry_ir`). **No real AIS** (only a synthetic truth-anchor in the `_anchored` variant). | lidar/radar seed + update; EO/IR are **bearing-only → corroborate only, cannot seed a Position2D birth** (the λ_C caveat, `pmbm-design.md §3.2.2`). |
| **philos** (Boston, real) | AIS-as-truth (sparse) | **radar/ArpaTtm only** (`philos_radar`), + **AIS on 2 clips** (`ais_ferry_near` 50 rows, `ais_ferry_far` 40). `PhilosScenarioRun` **never loads camera bearings or lidar** — no plumbing feeds them to the PMBM. | radar seeds/updates; AIS updates. |
| **harbor_complete_truth** (sim) | 5 (2 AIS movers, 3 radar boats) | pier + clutter are **radar-only**; the SUPPRESS set (pier) has no camera/lidar/AIS. | — |

**Harbor: A3 inert** (confirmed) — the pier over-count is a radar-only, chart-free
static structure; A3 has no clutter-free sensor to key on there. The harbor headroom
(`card_err 11.64→7.43`, charted-pier A/B) is the *static/charted* prior, a separate
Cl-4 rider — see `2026-07-10_harbor_truthsort_reconcile.md`.

## Step 1 — revival side (env-2): do the missed targets have the evidence?

`imm_cv_ct_pmbm_coverage_land` produces **zero tracks on all four env-2 scenarios**
(total collapse; per-scan state export). `imm_cv_ct_pmbm_land` (no no-birth floor)
tracks them — so the targets **are** trackable; the collapse is purely birth
suppression. Both targets are **in-band on 100 % of scans** (they hug the shore the
whole run — that is *why* they never birth).

Per-target in-band scans with clutter-free-sensor evidence (census, R=15 m/0.15 rad):

| scen | tgt | in-band | lidar | EO | IR | any camera | AIS | 2-sensor |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| sc16 | 1 | 1154 | 686 | 912 | 416 | 915 | 0 | 812 |
| sc16 | 2 | 1154 | 603 | 1154 | 979 | 1154 | 0 | 1108 |
| sc13 | 1 | 1152 | 725 | 954 | 928 | 1019 | 0 | 976 |
| sc13 | 2 | 1152 | 599 | 893 | 798 | 969 | 0 | 822 |
| sc17 | 1 | 777 | 273 | 775 | 410 | 775 | 0 | 587 |
| sc17 | 2 | 777 | 273 | 740 | 391 | 744 | 0 | 580 |
| sc22 | 1 | 854 | 367 | 786 | 214 | 786 | 0 | 635 |
| sc22 | 2 | 854 | 384 | 854 | 403 | 854 | 0 | 554 |

**Every target** has clutter-free evidence on **hundreds** of in-band scans (≫ the
3-scan revivability floor). Lidar alone — the only *birth-capable* clutter-free sensor
present — covers 35–63 % of in-band scans. AIS covers **0** (none present in the
no-AIS condition).

## Step 2 — guard side: what would each rule re-admit?

### 2a. Designated philos guard — STOP-AND-REPORT

The ticket's designated guard workload is philos, and it **cannot score the
camera/lidar variants**:

- **philos carries no lidar and no camera** on the bench path (census on
  `ais_ferry_near`: `ev_lidar = ev_cam = 0`; confirmed against the loader — no
  `loadCameraBearingsCsv`, no lidar). There is no plumbing by which a camera or lidar
  could lift or corroborate a near-shore birth on philos.
- Camera-bearing **fixtures** exist for only two clips. `ais_ferry_near` (which owns
  the "~185 near-shore returns" win) has **no shore-clutter labels**. `sunset_cruise`
  has labels but its camera bearings span only **[0°, +10°] off the bow** (dead ahead,
  at the real moving vessels), while the labelled shore clutter is off the starboard
  beam (`midriver_grp` ≈ +95°) and astern (`astern_blob` ≈ +198°) — **time-overlap
  yes, bearing-overlap no.** The camera never points at the clutter.

⇒ For a camera/lidar-corroborated rule, **there is no philos clip on which the camera
both sees the shore-clutter region and has a label there.** The guard side of the A/B
is unscoreable for these variants on this gauntlet. (AIS re-admission on philos is
scoreable and is ≈ 0 — AIS sits only on cooperative vessels, never on the fixed shore
clutter — but see K1.)

### 2b. env-2 self-guard — the only workload with cameras, and it is damning for camera

Because philos can't score camera/lidar, we measured the guard *density* on env-2
itself: of the in-band measurements from each clutter-free sensor, what fraction are
**off-target** (not within the gate of either real target) — i.e. structure/clutter a
sensor-typed exemption would ALSO admit:

| sensor | sc13 | sc16 | sc17 | sc22 | reading |
|---|---:|---:|---:|---:|---|
| **lidar** | 5 % | 5 % | 9 % | 7 % | **clean** — tight to real targets |
| radar | 50 % | 50 % | 64 % | 37 % | the shore-clutter artifact (as expected) |
| **EO camera** | 85 % | 85 % | 82 % | 79 % | **mostly structure** |
| **IR camera** | 80 % | 80 % | 84 % | 80 % | **mostly structure** |

Robust to gate width: doubling the gate (R=30 m/0.30 rad) leaves EO/IR at 72–78 % off
-target. **A camera-typed exemption re-admits ~80 % clutter** — this is exactly the
ADR §A3 "cameras see piers/background structure too" caveat, now measured. **Lidar,
by contrast, admits <21 % off-target** — a genuinely good near-shore discriminator on
env-2.

## Step 3 — score the variants against the binding kill-criteria

K1 (revival): revive ≥ 2/3 of env-2 in-band missed targets (≥3 corroborated scans
each). K2 (guard): philos re-admission ≤ 10 % of what A1 re-admitted (A1 re-admitted
the full residual: gospa 73.1→100, card_err +6.9→+36.2 — first-order, scale linearly
by re-admission fraction).

| variant | K1 (env-2 revival) | K2 (philos guard) | build? |
|---|---|---|---|
| **(i) AIS-only exempt** | **FAIL** — 0/8 targets (no AIS in the no-AIS condition, the deployment-relevant one) | PASS (AIS re-admits ≈0 shore clutter → projected gospa ≈ 73.1) | **NO** — split verdict: passes the guard but can't revive the deployment condition |
| **(ii) AIS + camera(+lidar)** | **PASS** — 8/8 targets (lidar birth-capable; camera corroborates) | **UNSCOREABLE on philos** (no camera/lidar there). *Camera component measured-dead on env-2: ~80 % off-target → projected re-admission ≈ full residual → gospa ≈ 100 if it transfers.* Lidar component clean on env-2 but philos has no lidar to test. | **NO** — guard cannot be certified; camera component fails its env-2 self-guard |
| **(iii) two-sensor corroboration** | **PASS** — 8/8 targets | **UNSCOREABLE on philos.** Dominant 2nd sensor is camera (80 % off-target); corroboration with a clutter-heavy camera is unsafe and can't be verified on the guard workload | **NO** |

**No variant qualifies for BUILD.** No variant meets BOTH K1 and K2:

- AIS-only is the only variant whose guard is honestly scoreable, and it **passes K2
  but fails K1** on the deployment-relevant no-AIS condition. (Exactly the split the
  ticket anticipated.)
- The variants that pass K1 (camera/lidar) have a guard that is **unscoreable on
  philos** — and where cameras *are* measurable (env-2), the camera exemption fails
  its own guard at ~80 % off-target.

### Why this is a real data limitation, not a tuning shortfall

The revival evidence (env-2 lidar/camera) and the guard workload (philos radar shore
clutter) **share no clutter-free sensor** on the deployment-relevant condition:

- env-2 has lidar + camera but no AIS (no-AIS condition); philos has radar + sparse
  AIS but **no lidar and no camera**.
- So a lidar/camera rule can be *revival-tested* (env-2) but not *guard-tested*
  (philos has neither); an AIS rule can be *guard-tested* (philos) but not
  *revival-tested* (env-2 no-AIS has no AIS).

A3 as specified — "exempt births seeded by clutter-free sensors" — therefore cannot be
certified as the **one deployable config** on this gauntlet: the one clean
discriminator (lidar) is present only in Trondheim, the one cooperative sensor (AIS) is
absent exactly where revival is needed, and the camera is measured to be a clutter
source, not a discriminator.

## Recommendation

**NO-BUILD for A3 in Phase 1.** Pivot to ranked path (b) — the **conditional coverage
floor / re-detection-based near-shore birth** (ADR §A2): a re-detected near-shore
return ramps past the gate while one-shot clutter does not. That path does **not**
depend on a cross-workload clutter-free sensor and so is not blocked by this data gap.
The arbiter writes the path-(b) probe.

Two durable facts for the backlog:
1. **Camera is a clutter source near shore, not a discriminator** — 79–85 % of in-band
   env-2 camera bearings are off-target (gate-robust). Any future camera use near shore
   must be corroboration-gated, never a standalone exemption.
2. **A3's revival evidence is real** (lidar + camera present and on-target for the
   env-2 targets, 100 % in-band). If a deployment ever carries lidar *and* the near
   -shore clutter workload also carries lidar, A3-via-lidar is worth revisiting — but
   that is a sensor-fit question, not a Phase-2 build on today's gauntlet.

## Reproduce

```
# build the census tool
cmake --build build --target navtracker_cl4_a3_census
# env-2 revival + self-guard (per scenario)
./build/bench/navtracker_cl4_a3_census --scenario autoferry_scenario16
# collapse confirmation (zero tracks) / survivor comparison
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_coverage_land \
   --scenario-eq autoferry_scenario16 --seeds 1 --export-states-dir <dir>
./build/bench/navtracker_bench_baseline --config-eq imm_cv_ct_pmbm_land \
   --scenario-eq autoferry_scenario16 --seeds 1 --export-states-dir <dir>
```
Fixtures (`data/autoferry`, `tests/fixtures/philos/out`) are symlinked from the main
tree; they are gitignored and not part of the commit.
