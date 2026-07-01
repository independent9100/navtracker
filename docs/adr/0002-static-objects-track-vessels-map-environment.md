# ADR 0002 — Static objects: track vessels, map the environment

- **Status:** Accepted
- **Date:** 2026-07-01
- **Deciders:** navtracker maintainers
- **Related:** ADR 0001 (near-shore no-birth zone); design spec `docs/superpowers/specs/2026-05-28-maritime-sensor-fusion-design.md` §2 / §14.3 / §14.10; `docs/algorithms/evaluation-log.md` (2026-07-01, "philos radar reality" + "open-sea missed targets"); `docs/learning/25-land-clutter-prior.md`; `core/land/CoastlineModel.hpp`; `core/tracking/ClutterMapDetectionModel.hpp`; `core/output/TrackOutput.hpp` / `docs/output-contract.md`

## Context

Two threads converged on one question: **should navtracker track static objects?**

### What we measured (the insight)

The philos (Boston Harbor) replay's chronic PMBM **over-count** — the phantom
tracks the land prior exists to suppress — was investigated against the **raw
radar**, not the AIS-only truth (2026-07-01; see eval log for the full run):

- The radar plots are **raw detections**, produced by a custom offline chain
  (intensity threshold → DBSCAN cluster of the `radar_pcd` point clouds;
  `tests/fixtures/philos/extract_radar.py`). They are **not ARPA tracks** — no
  track id / course / speed / CPA, ~10 plots per sweep.
- They are **near-field**: every return is ≤ ~976 m from own-ship, and **only 1
  of the 23 AIS "truth" vessels is within radar range** (77 m); the other 22 are
  1.2–15.8 km away (beyond the plots). So on this clip the radar barely overlaps
  the scored targets — it is dominated by near-field returns.
- The ~1,940 non-AIS returns form a ring of **persistent, fixed, extended
  structure** (piers / breakwaters / shoreline; `n_cells` up to 6181;
  straight-line features). A motion test (tracklet linking + a
  time-monotonicity filter) found **zero** coherent non-AIS *moving* boats; the
  only "coherent" tracks it produced were impossible-speed (12–86 m/s) artifacts
  of a naive linker chaining returns *along* a fixed structure.

Conclusion: the philos over-count is **static infrastructure**, and philos is a
realistic, valuable **clutter-rejection / false-positive** test — not a
radar+AIS fusion test.

### The realization

"Static" is two different things, and conflating them is the trap:

- A **static-but-real vessel** (anchored / moored / drifting) is a *vessel* with
  ~zero SOG. It is a hard collision hazard, may get underway at any instant, and
  is already representable — the output contract defines a *"Confirmed
  stationary; direction undefined"* state (`docs/output-contract.md`), and the
  CV CPA math reduces correctly for a zero-velocity target.
- **Fixed infrastructure** (piers, breakwaters, shoreline) is *environment*: an
  extended, many-cell object with no velocity, known a priori from charts, for
  which a point-target CV filter and center-to-center CPA are structurally
  wrong.

**The decision rule is therefore vessel-vs-environment, not
moving-vs-stationary.** SOG≈0 is a weak, dangerous discriminator: it fails on
exactly the case that matters most — the anchored vessel.

### Why this cannot simply be pushed "elsewhere"

navtracker is the fusion hub: it already holds every sensor, the chart,
own-ship state, **and** the confirmed dynamic tracks. No downstream entity
knows more. So a static-obstacle capability — especially for an **uncharted**
obstacle (a pillar in the water on no chart) — must live *inside* navtracker,
because only navtracker has the information to detect it.

### The gap this exposes (grounded in code)

navtracker's only near-shore discriminator is "persistent radar return near
shore," which is identical for a moored vessel and a pier:

- Under `coverage_land` / `bundle_land`, a non-cooperative vessel anchored
  within ~50 m of shore never initiates a track (ADR 0001 no-birth cliff). The
  2026-07-01 fix `imm_cv_ct_pmbm_land` (adapt + land prior, **no**
  `birth_existence_target`) largely relaxes this: with the gate back at 0.05 and
  no birth pin, a near-shore vessel still births through the soft ramp; only the
  inland hard gate blocks it.
- The IMM has **no stationary mode** (`makeImmCvCt` = CV5State +
  CoordinatedTurn, both accel PSD 0.5, `kImmInitSpeedStd = 10`): a moored track
  is held but with inflated covariance (wider gate → more clutter admitted), and
  the coordinated-turn mode's turn-rate is unobservable at v≈0.
- The spatial **clutter map is inert in PMBM** (`observe()` is only called by
  `MhtTracker`), so it neither helps nor deletes an anchored vessel there — but
  its persistent-cell primitive is the seed of a proper static-occupancy
  estimator.

## Decision

1. **The vessel MTT core (PMBM/MHT point-target tracker) tracks discrete
   VESSELS, including static ones (anchored / moored / drifting). It does NOT
   emit fixed infrastructure as vessel tracks.** "Static" is a track
   attribute/state, never a reason to drop a real vessel.

2. **Static-obstacle handling is a separate branch — a distinct estimator —
   *inside* navtracker**, not a downstream concern. It consumes the same fused
   measurements + own-ship pose + chart, estimates *environment occupancy /
   static hazards* (not kinematic tracks), and is combined with the vessel
   tracks in a world-model / output step. Both feed the collision layer.

3. **The two branches cross-feed.** Vessel tracks label returns already
   explained by a confident target (weight `1 − r`) so the static branch does
   not double-count them; the static-occupancy estimate acts as a vessel-birth
   prior that suppresses phantom tracks. This is the honest generalization of
   today's land prior + clutter map.

4. **Charted static obstacles are supplied as a separate input (`StaticObstacle`),
   distinct from the coastline geometry** — discrete point / small-polygon
   hazards with a keep-clear radius and attributes (type, depth, lit,
   real/virtual AtoN), not folded into the land polygon.

## Why (rationale)

**Different estimator, not the same filter doing double duty.** The two problems
are structurally incompatible:

| | vessel branch (have) | static branch (this ADR) |
|---|---|---|
| measurement model | point target, ≤1 detection/scan, Poisson clutter | extended, many-cell, spatially continuous |
| state | kinematic point (pos, vel, turn-rate) | occupancy of space / extent — no velocity |
| lifecycle | birth → confirm → delete by kinematic gating | evidence accumulates per cell over time |
| output | CPA/TCPA (for maneuver) | keep-clear distance / no-go region |

Feeding a 6181-cell structure into a point-target filter is what *produces* the
philos over-count; center-to-center CPA to one point on a wall is meaningless
(you pass the centroid at 200 m while the wall is 20 m off your beam). Extended-
object tracking is already scoped as its own project (spec §14.3).

**No other entity has the information.** For an uncharted obstacle, navtracker is
the only place with the fused sensors + chart + own-ship + dynamic tracks, so
the capability belongs here — as a sibling branch, sharing inputs.

**Cross-feed is the payoff of co-location.** The branches make each other
better; that is only possible in one process.

**Reframe of existing work.** The land model is the *chart-based* static prior;
the clutter map is a crude *live* static-occupancy estimator. The static branch
unifies them honestly — and turns the "clutter" we keep fighting into an
explicit hazard output.

**Charted obstacles separate from the coastline** because the semantics differ:
a shoreline is a large region you never go behind (a birth-suppression ramp); a
pillar/rock/wreck/dolphin is a *discrete* hazard in navigable water you keep
clear of but navigate *around*. Folding obstacles into the land polygon creates
a tiny no-birth zone around each — suppressing a real vessel passing close (the
same anchored-vessel trap) — and loses obstacle attributes a polygon cannot
carry.

## Consequences

**Positive**
- The vessel track picture stays clean (no infrastructure pollution / phantom
  cardinality).
- Static hazards, including **uncharted** ones, get an honest home.
- Three things we have been fighting — the land prior, the clutter map, the
  philos over-count — collapse into one coherent capability.
- The `StaticObstacle` chart input enables a cheap first stage immediately.

**Negative / gaps to close (the anchored-vessel safety work)**
- Near-shore static-vessel birth needs the **sensor/chart-aware discriminator**
  (ADR 0001 A3): radar-only + chart-coincident → suppress; camera/AIS
  corroboration or a compact swinging (watch-circle) return → birth a vessel.
  Partly addressed by `imm_cv_ct_pmbm_land`; finish via sensor-aware suppression.
- Add a **stationary IMM mode** (low-PSD "stopped" mode, or a zero-velocity
  pseudo-measurement gated on |v| < threshold) so a moored track's covariance
  stays tight and CPA gets a clean stationary-hazard flag + getting-underway
  transition.
- Add a scenario test: a target that **starts anchored (v≈0), holds, then gets
  underway** — assert it initiates, is not suppressed, and keeps a **stable
  track_id** through the stop→go transition (unverified today).
- Document that navtracker's **CPA covers own-ship × vessel-track only**;
  infrastructure collision warning is the static branch's job, so consumers do
  not over-trust the collision layer for walls.
- A full evidential/occupancy grid is a substantial new subsystem — stage it.

## Alternatives considered

- **Track everything, including infrastructure, as vessel tracks.** Rejected:
  extended-object / point-target mismatch → phantom flood (the philos
  over-count), meaningless CPA, corrupted cardinality and non-actionable
  collision alerts.
- **Push static-obstacle handling entirely outside navtracker.** Rejected: no
  downstream entity has navtracker's fused information; an uncharted obstacle can
  only be detected here.
- **Fold charted obstacles into the coastline polygon.** Rejected: wrong
  semantics; creates tiny no-birth zones around each obstacle that suppress
  vessels passing close; loses obstacle attributes.
- **Keep the clutter map as a silent λ_C clutter hack.** Rejected: it is inert
  in PMBM and semantically wrong; the same persistent-cell primitive is valuable
  when reframed as an explicit static-occupancy *output*.

## Staging

1. **Cheap, now:** the `StaticObstacle` chart input as a vessel-birth prior +
   hazard output; reframe the clutter-map primitive into an explicit
   "persistent-unclaimed-return → static-occupancy" layer, wired into PMBM
   properly (the parked "PMBM feeds the clutter map via dominant-hypothesis
   `1 − r` labeling" design) and **output as a hazard**, not a hidden λ_C tweak.
2. **Later:** a proper Bayesian / evidential occupancy grid for full
   uncharted-static mapping; the stationary IMM mode; sensor-aware near-shore
   birth.

## Prior art & validation (2026-07-01 literature + standards + commercial sweep)

The decision was checked against academic tracking theory, maritime standards,
robotics autonomy, and commercial systems. **Verdict: ADR 0002 is validated on
all four fronts, and is AHEAD of the standards baseline on one axis** (in-process
detection of *uncharted* statics + active cross-feed).

- **Academic (RFS/MTT).** Feeding an extended, many-cell return into a
  point-target filter is the *textbook* failure mode — our philos over-count is
  the documented symptom. The field offers three treatments and we sit in the
  modern one: (A) suppress as structured clutter (clutter maps / non-homogeneous
  λ_C — our *current* hack); (B) extended-object tracking (GGIW random matrices,
  GP extent, PMBM-for-extended); (C) **map statics separately and cross-feed** —
  SLAMMOT (Wang, Thorpe, Thrun & Durrant-Whyte, IJRR 2007) *proves* that
  decomposing into two estimators (static map + moving tracker) is the tractable,
  consistent architecture, and DOGMa (Nuss et al., IJRR 2018, arXiv:1605.02406)
  is an RFS occupancy grid in the *same filter family as PMBM* — so our two
  branches are mathematically compatible siblings. **Near-exact maritime prior
  art (verified against the full text):** Herrmann, García-Fernández, Brekke &
  Eide, "A Scalable Hybrid Track-Before-Detect Tracking System: Application to
  Coastal Maritime Radar Surveillance" (arXiv:2508.16169, 22 Aug 2025; for IEEE
  J. Oceanic Eng.) — real X-band coastal radar (Trondheim Fjord). It runs a
  conventional PMBM point tracker on **DBSCAN**-clustered high-threshold
  detections, removes land with a **separate precomputed median land mask**
  (offline, thresholded, morphologically dilated — a static-map layer applied
  before tracking), and pairs it with an IE-PHPMHT track-before-detect module for
  weak targets. Critically it **tracks stationary fishing vessels that carry no
  AIS "also tracked by the PMBM component"** — a leading group's real-radar
  endorsement of our exact decisions (PMBM point tracker for vessels incl.
  stationary/non-AIS ones; environment as a separate mask/map layer). The one
  place they go beyond us is the TBD channel for weak targets (a future option
  for navtracker).
- **Standards (IMO/IHO/IALA).** The standards architecture *is* ADR 0002:
  charted statics live in the ENC as typed objects (S-57 `OBSTRN` / `UWTROC` /
  `WRECKS` / `SLCONS` / `PILPNT`, S-101 successor) + AIS AtoN (Message 21, incl.
  virtual); vessels — including anchored/moored — are tracked targets under
  COLREGS; the two are separate INS task modules combined downstream
  (MSC.252(83)). IHO keeps discrete hazards as classes *distinct from* the
  coastline (`COALNE`/`LNDARE`) — directly validating our decision 4
  (`StaticObstacle` separate from the coastline). An anchored vessel is
  unambiguously a *vessel* (COLREGS Rule 30; AIS nav-status at-anchor=1/moored=5
  are *attributes* in the vessel position report, never re-broadcast as AtoN) —
  a standards-level endorsement of "vessel-vs-environment, not
  moving-vs-stationary." **We are ahead:** standards assume statics are
  pre-charted or broadcast and specify *no* onboard estimator for *uncharted*
  statics — exactly the pillar-in-the-water case our branch adds.
- **Commercial.** Classic ARPA (Furuno/JRC/Simrad/Kongsberg) *suppresses*
  statics rather than understanding them — signal-level clutter control
  (STC/CFAR), guard-zone *exclusion* areas, Furuno's "ARPA Land Discrimination"
  — and opportunistically uses one stationary echo as a ground-speed reference
  (IMO A.823(19)). Its documented near-shore failure (an anchored vessel lost to
  anti-clutter / land discrimination near shore) **is our ADR 0001/0002 trap,
  confirmed as real commercial behaviour.** Modern maritime-AI (Kongsberg
  SeaAware, Sea Machines SM300, Orca AI, Sea.AI) fuse chart + radar + EO/IR +
  AIS and *do* detect fixed structures / buoys / non-AIS craft — our modern
  stance matches theirs (we are behind on vision/LiDAR sensor breadth, not on
  architecture).
- **Robotics/autonomy.** Our split is the *mainstream* architecture
  (SLAMMOT / DATMO-alongside-SLAM), not a bespoke choice. Nobody ignores statics
  — statics *are* the map; they're just kept out of the track *list*, and
  dynamic returns are actively removed from the static map (LiDAR-MOS) — the
  reciprocal of our `1 − r` labeling. The anchored-vessel case is the famous
  *stopped/parked-car* problem: velocity-based grids absorb a stopped car into
  the static layer and drop it — the exact failure our rule guards against;
  the fix is object-ness/semantics, not velocity.

**"Do some just ignore statics?"** Yes — classic ARPA and clutter-map/λ_C
methods suppress them, and GMTI notches out zero-Doppler scatterers. That is the
old tradition, and it is unsafe once something you care about is stationary. We
deliberately take the modern *map-and-output* stance.

### Refinements adopted into the plan (from the sweep)

1. **Schema-align `StaticObstacle` to the ENC / S-101 model** — carry `CATOBS`
   (category), `WATLEV` (submerged/awash/covers-uncovers), `VALSOU`/depth,
   `EXPSOU` (vs safety contour), lit, a real/synthetic/virtual-AtoN flag, and a
   positional-uncertainty **buffer radius** (the principled basis for
   "keep-clear radius"). Lets us ingest ENCs directly and reuse an
   internationally-agreed hazard ontology instead of inventing one.
2. **Make the stage-2 static branch a Dempster-Shafer *evidential* occupancy
   grid** (masses for free / static-occupied / dynamic-occupied / **unknown**),
   i.e. DOGMa in our own RFS family. The explicit **unknown** state is the
   honest fix for ADR 0001's hard no-birth cliff (carry "unobserved," don't
   blanket-suppress), and inter-scan DS *conflict* is a chart-free
   static-vs-clutter-vs-dynamic discriminator — the honest form of the
   "reframe the clutter map into a hazard" plan.
3. **Adopt AIS Message 21 (incl. virtual AtoN) as a first-class
   `StaticObstacle` source**, and treat AIS nav-status 1/5/6 as a vessel-track
   *attribute* hint feeding the "Confirmed stationary" state — never a
   delete/reclassify trigger.
4. **Benchmark against Herrmann et al. 2025** as the closest published system.

## Revisit when

- A deployment needs uncharted-static-obstacle avoidance → build stage 2.
- EO/IR or AIS coverage near shore becomes available → enables the sensor-aware
  discriminator and unblocks reliable near-shore static-*vessel* tracking.
