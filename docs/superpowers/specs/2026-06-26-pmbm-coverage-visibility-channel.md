# PMBM coverage / visibility channel — design spec (Task 4)

Status: design, not yet implemented. Date: 2026-06-26.
Supersedes the one-paragraph "Task 4" in
`docs/superpowers/plans/2026-06-24-pmbm-philos-cardinality-improvements.md`.

This spec is written in plain words on purpose. It is the design the
PMBM tracker needs so it can decide "is this track still real?" using
an honest model of *which sensor could have seen what, and when* —
instead of the broken shortcuts it uses today.

---

## 1. The problem, in one breath

When a target is *not* seen, PMBM lowers that track's "I still exist"
number. To do that correctly it must know: **did a sensor actually
have a real chance to see this target just now, and did that chance go
by with nothing?**

Today PMBM cannot answer that. Three reasons:

1. **There is no real "sweep" concept.** A "scan" in the code is just
   *"all measurements that happen to carry the exact same timestamp"*
   (`core/benchmark/BenchRunner.cpp:257-264`). It is not a radar
   rotation, not a sensor cycle, not a time window. So a sensor only
   counts as "looked and saw nothing" if it *happened to emit some
   other measurement at the same instant*. That is an accident of
   batching, not a model of coverage.
2. **The miss-penalty math is wrong and load-bearing.** `compute_miss_pD`
   multiplies the penalty once per blip instead of once per sweep
   (`core/pmbm/PmbmTracker.cpp` miss branch). It is the only thing
   currently stopping PMBM from drowning in phantom tracks — a crutch.
3. **The "no contact for a while → fade out" knob (`idle_halflife_sec`)
   is a guess, not a model.** One global fade rate for every target,
   regardless of how often that target is actually expected to report.

The result on the `philos` replay: PMBM over-counts (phantom tracks),
because it has no honest way to tell *"gone"* from *"nobody looked"*.

**Goal of Task 4:** give PMBM a real coverage model so it can tell
those apart — and in doing so **retire all three crutches** (the wrong
miss-math, the `idle_halflife` hack, and the `source_id="ais"` patch
from Task 2a).

---

## 2. The one idea everything hangs on

For every pair of *(channel, track)* ask one question:

> **"When did this channel last have a *real chance* to observe this
> track, did that chance pass with nothing, and how strong is that
> 'nothing' as evidence the track is gone?"**

Different sensors fill in that question very differently. There are
exactly **two kinds** of sensor, and they are opposite in nature:

### Kind A — Surveillance sensors (radar, EO/IR, lidar)
They **search** an area on a rhythm *you* control.
- A "chance to see" = the beam/scan swept over the track's location and
  finished that sweep. Its rhythm is the **duty cycle** (e.g. radar at
  60 s per rotation = one chance every 60 s).
- Silence over ground it *did* cover is **strong** evidence: "nothing
  is there." This works even for targets you have never seen before.
- Evidence is roughly **symmetric**: a detection and a covered-but-empty
  sweep are both informative.

### Kind B — Cooperative-announce sources (AIS, and the Cooperative channel)
The target **announces itself**; you do not search for it.
- A "chance to hear" = the target's *own* expected report time arrived.
  The rhythm belongs to the **target**, not to you.
- Silence is **weak** evidence. A quiet transponder/partner does not
  mean the vessel sank — it may have switched off, dropped out of
  range, or hit congestion.
- Evidence is **asymmetric**: a report is *strong* (near-certain the
  target exists, here, precisely, with an identity); silence is *weak*.
- It is keyed on **identity**, not on a place. Vessel B's silence says
  nothing about vessel A, and nothing about a radar-only track.

**AIS and the Cooperative channel are the same kind (B), on a
trust/reliability dial:**

| Axis | AIS | Cooperative channel |
|---|---|---|
| Trust of a report | spoofable, can be wrong/stale | authenticated/encrypted → take at face value |
| Link reliability | congestion, range, switched off | dedicated, reliable, few drops |
| Roster | open population of all vessels | known, small (your own partners) |
| Cross-check vs radar? | yes (trust-but-verify) | no — believe it, even over a radar disagreement |
| Expected report rate | variable, must be inferred | predictable, can be declared |

So the Cooperative channel is just the **gold-standard end** of the
cooperative kind. We do **not** add a third category.

---

## 3. The new port: `ISensorActivity`

A small interface (zero I/O, lives in `ports/`) that answers the
"real chance to observe" question. It is **nullable**: if nobody wires
it, PMBM behaves exactly as today.

It must answer, for a given track position / identity and a given time:

- **For a surveillance sensor:** *Is this sensor switched on, and does
  its coverage (range + sector) include this track's position, and has
  it completed a sweep since we last checked?* If yes and no return
  came → one miss opportunity.
- **For a cooperative source:** *Was this specific track's own report
  expected by now (per its cadence) and did it not arrive?* If yes →
  raise a **comms-loss / stale signal** for that track — **not** an
  existence penalty (decision §9c: lost comms ≠ sank). See §4.

The port carries, per sensor/source, a small **profile**:
- `kind`: Surveillance or Cooperative.
- coverage: reuse what already exists — `max_range_m`, azimuth sector,
  `missDetectionProbability(...)` in `ports/ISensorDetectionModel.hpp`.
- **cadence**: the duty-cycle period (surveillance) or the expected
  report interval (cooperative).
- **trust/strength**: how hard a confirmed surveillance miss may push
  existence down. (Cooperative misses do not touch existence — see §4.)

**Cadence comes from an exchangeable provider (decision §9a).** The
profile lookup is itself an interface so the source of cadence/coverage
is pluggable. Implementation #1 is a **declared profile** (static,
deterministic config). An **adaptive, learned-per-source** profile is a
planned later implementation behind the same interface — see the
Roadmap (§13). This keeps the declared model simple now without
boxing out the adaptive model later.

---

## 4. How the misdetection step changes (the math, plainly)

Existence after a miss uses the standard Bernoulli recursion:

```
r_after = (1 − p_D)·r / (1 − r·p_D)
```

Higher `p_D` ⇒ bigger drop. The whole point is to feed it the **right
`p_D`, charged the right number of times.** Today it is charged once
per blip (wrong). New rule:

**Charge at most one miss opportunity per sensor *duty cycle*, not per
timestamp-batch.**

Step by step, for each track and each channel the activity port knows:

1. **Surveillance channel.** If the sensor is active, its coverage
   includes the track's predicted position, and it has completed a
   sweep with no associated return for this track → apply **one** miss
   with that sensor's `p_D`. If the track is outside coverage, or the
   sweep has not completed yet → apply **nothing** (no penalty).
2. **Cooperative channel — does NOT touch existence (decision §9c).**
   If this track's *own identity* was expected to report by now and did
   not, raise a **comms-loss / stale signal** for the track and leave
   its existence alone. Rationale: comms are routinely unreliable and
   connection losses are real, so a silent fleet member means "we lost
   the link," not "the vessel sank." So:
   - A surveillance-held track that goes quiet on its cooperative link
     → existence untouched (radar carries it), flagged stale.
   - A **cooperative-only** track (no surveillance) that goes quiet →
     existence still untouched; it enters a **stale/coasting** status
     surfaced to the operator, and is retired **only** by a separate,
     long, explicit max-stale timeout (operator-tunable) or once a
     surveillance sensor that *does* cover its position confirms
     absence. It is never killed by the per-sweep miss math.
3. **No surveillance channel had a real chance** (between sweeps,
   target out of all coverage) → existence is **unchanged**. This is
   what replaces `idle_halflife_sec`: tracks do not bleed out on a
   wall-clock timer; existence drops **only** from a genuine
   surveillance miss (case 1).

This automatically fixes the old "inconsistent P_D" bug too: the
detect branch and the miss branch now use the same per-sweep `p_D`.

---

## 5. Identity — cooperative carries its own unique id; MMSI is an optional bridge

**The two facts (from the domain owner):**
1. Every cooperative (fleet-member) message carries **its own
   identifier, which can be assumed unique.** This is the *primary*,
   stable identity for cooperative tracks — use it directly.
2. Some fleet members also run AIS and **could additionally** include
   their MMSI on the cooperative message. Optional.

**Current state:** there is no home in `AssociationHints` for a
cooperative-native id today. `hints.sensor_track_id` is the wrong
field — it is only *set* by the ARPA / EO-IR adapters, **never read by
the core**, explicitly "an opportunistic cue, never the fusion key", an
ARPA target number that gets reused/renumbered on drop-reacquire, and
unique only *within one sensor*. So it is **not** stable. `hints.mmsi`
is the existing per-vessel id but is AIS-specific (`uint32`).

**Design:**
- **Add a typed cooperative identity to `AssociationHints`** —
  `std::optional<std::uint64_t> platform_id;` (numeric, settled
  2026-06-29). The Cooperative adapter **always** sets it. Task 4's cadence logic and
  the identity gate key on this for cooperative tracks. Because it is
  reliably unique it is a strong association prior — but still a
  **hint, never the fusion key** (invariant 5); `track_id` stays
  primary.
- **MMSI stays optional and additional.** When a fleet member also has
  AIS, the adapter *also* sets `hints.mmsi`. That MMSI is the **bridge**
  that links the member's cooperative fix to that same vessel's AIS
  broadcasts.
- **Generalise Task 2a's gate** to a unified key: a track's identity is
  `mmsi` when present, else `platform_id`; two observations are the same
  vessel if they share **either** `mmsi` **or** `platform_id`. That is
  what makes the cooperative+AIS fusion happen when MMSI is supplied,
  while cooperative-only members still work on `platform_id` alone.

**Is the optional MMSI worth including? — Recommendation: yes, when
available, but it is an enhancement, not a Task-4 dependency.**
- Task 4's coverage/cadence works on `platform_id` **alone**; it never
  needs MMSI.
- MMSI buys exactly one thing: it fuses a fleet member's *cooperative
  fix* with its *AIS broadcasts*. Without it, a member that is both a
  cooperative partner **and** on AIS may be carried as **two separate
  tracks** for one real vessel (or merged only by shaky spatial
  gating). With it, they fuse by identity, and the high-trust
  cooperative fix cross-validates the AIS report (catching AIS
  error/spoof). Cheap to populate, strictly additive.

**Settled (2026-06-29):** `platform_id` is **numeric**
(`std::uint64_t`).

---

## 6. Input contract (what the consumer feeds us)

Keep it event-driven. Two firm rules:

1. **Never re-feed stale positions.** Do not periodically re-send a
   3-minutes-old fix as if it were fresh — it would corrupt the
   estimator and break determinism. Measurements stay real and
   timestamped at the moment they were produced.
2. **Coverage/cadence rides a separate channel — the activity port.**
   The consumer declares "radar is active, here is its rotation period
   and coverage" and "this partner reports about every N seconds",
   either as **config** (a declared profile) or as lightweight
   **activity events** (e.g. "sweep boundary at time t"). The tracker
   reasons about silence from *that*, not from faked data.

Determinism is preserved: the activity port is a pure function of
declared profiles + timestamps; no wall-clock, no RNG.

---

## 7. Birth confidence by kind — DEFERRED to roadmap (decision §9b)

The same surveillance/cooperative split could also set how confident a
**new** track is:

- **Surveillance birth (radar blip): timid.** Might be clutter (the
  whole philos shore-return problem). Born weak, earns confidence over
  later detections — this is exactly Task 1.
- **Cooperative birth: confident.** A self-report is almost certainly a
  real vessel.
- **Cooperative-channel birth: the most confident of all** —
  authenticated, no clutter, no spoof to defend against. Can be born
  effectively confirmed on first contact.

**Decision: defer this out of Task 4** (see Roadmap §13). Reason: the
philos cardinality problem is driven by radar *over*-births, which
**Task 1 already fixes** (clutter-invariant timid births). Cooperative
under-confidence is not the current pain, and a cooperative track that
is born timid simply confirms after a couple of reports — acceptable.
Keeping birth-confidence-by-kind separate keeps Task 4 focused on the
coverage/visibility model. It is recorded as a near-term roadmap item.

---

## 8. Required four-section algorithm doc (CLAUDE.md standard)

**Math.** Bernoulli existence recursion
`r⁺ = (1−p_D)·r / (1−r·p_D)`, `p_D` resolved per *duty cycle* per
**surveillance** channel (not per blip): full `p_D` from
`missDetectionProbability` when active + in-coverage + sweep complete;
no coverage chance → `p_D = 0` → `r` unchanged. **Cooperative channels
do not enter this recursion at all** — an overdue cooperative report
raises a comms-loss/stale signal and leaves `r` untouched (decision
§9c).

**Assumptions.** (1) Each surveillance sensor has a known coverage
(range/sector) and a known duty-cycle period. (2) Each cooperative
source has a per-track expected report interval. (3) Cooperative
identity is stable enough to key cadence (it is a hint, not the fusion
key). (4) The consumer supplies activity/coverage via the port, not by
re-feeding stale measurements.

**Rationale.** Today "scan" = same-timestamp batch, which makes the
miss penalty accidental and forces two crutches (wrong miss-math,
`idle_halflife`). Charging misses on each sensor's real duty cycle is
the textbook "one sweep = one opportunity" model and is what MHT
already approximates with its IPDA/VIMM visibility channel
(`core/tracking/TrackTree.cpp` visibility branch). Modelling AIS and
the Cooperative channel as *announce* sources (asymmetric evidence)
rather than *search* sources is what lets us stop punishing a track for
a quiet transponder.

**Ways to improve / what to test next.** See the Roadmap (§13): the
adaptive cadence provider, birth-confidence-by-kind, and
target-dependent `p_D` (small vessels are genuinely harder to detect —
an RCS/size term) are all recorded there.

---

## 9. Decisions (settled 2026-06-26)

- **(a) Cadence source → DECLARED PROFILE, behind an exchangeable
  interface.** Cadence/coverage is supplied by a declared static config
  (simple, deterministic) read through a pluggable interface, so an
  adaptive learned-per-source model can replace it later without
  touching the tracker. Adaptive model recorded in Roadmap §13.
- **(b) Birth confidence by kind → DEFERRED.** Kept out of Task 4 (Task
  1 already fixes the radar over-birth that drives philos). Recorded in
  Roadmap §13. See §7.
- **(c) Cooperative-overdue → COMMS-LOSS SIGNAL, not existence decay.**
  Lost comms ≠ sank; connection losses are real and routine. An overdue
  cooperative report raises a stale/comms-loss signal and does **not**
  enter the existence recursion. A cooperative-only track is retired
  only by a long explicit max-stale timeout or surveillance-confirmed
  absence. See §4 case 2.

All choices settled. `platform_id` is numeric (`std::uint64_t`),
settled 2026-06-29 (§5).

---

## 10. Implementation outline (TDD, one commit per step group)

Build/bench protocol and autoferry guard: reuse the "Shared protocol"
section of the Task 1–7 plan.

- [ ] **Step 1 — the port + cadence provider.** Create
  `ports/ISensorActivity.hpp` (pure interface, documented
  surveillance-vs-cooperative split, the profile fields from §3). Make
  the profile/cadence lookup an **exchangeable interface** (decision
  §9a) with a **declared-profile** implementation; an adaptive impl
  slots in later behind the same interface (Roadmap §13). No I/O.
- [ ] **Step 2 — wire it in.** Add
  `PmbmTracker::setSensorActivity(const ISensorActivity*)` (nullable;
  null = today's behaviour, bit-identical).
- [ ] **Step 3 — identity (§5).** Add `platform_id` to
  `AssociationHints`; the Cooperative adapter always sets it from the
  fleet-member id, and *also* sets `hints.mmsi` when the member has
  AIS. Generalise Task 2a's gate to key on `mmsi` else `platform_id`,
  and to treat two observations as one vessel if they share *either*.
  Keep `source_id="ais"` unchanged (MHT relies on it). Unit-test:
  (a) a cooperative-only track is keyed by `platform_id`; (b) a member
  carrying both `platform_id` and `mmsi` fuses with its AIS track.
- [ ] **Step 4 — surveillance miss (TDD).** Test: a radar that is
  active and covers a Bernoulli but returns nothing this sweep applies
  exactly **one** miss; outside coverage / mid-sweep applies none.
  Implement the per-duty-cycle miss in the PMBM miss branch. See it
  pass.
- [ ] **Step 5 — cooperative stale signal (TDD), §9c.** An overdue
  cooperative report **must not change existence** — it raises a
  comms-loss/stale signal instead. Decide the signal mechanism (a new
  optional sink, or a track status/attribute — mirror the existing
  `ITrackSink` pattern). Tests: (a) an overdue own-identity report
  leaves `r` unchanged and flags the track stale; (b) a cooperative
  source never affects a *different* identity's track; (c) a
  cooperative-only track is retired only by the long max-stale timeout
  (or surveillance-confirmed absence), never by the miss math.
- [ ] **Step 6 — retire the crutches.** With the port wired on a
  config, turn **off** `idle_halflife` decay and the wrong-math
  per-blip miss path; confirm existence now moves **only** on a genuine
  surveillance miss (§4 case 1). Keep the old paths default-on for
  back-compat (new behaviour behind a flag/config).
- [ ] **Step 7 — philos activity model.** In
  `adapters/benchmark/ReplayScenarioRun.cpp` supply a philos activity
  profile: radar = surveillance with its rotation cadence + coverage;
  AIS = cooperative per-vessel; (Cooperative channel if present in the
  fixture). Bench philos; A/B against the Task-2 bundle.
- [ ] **Step 8 — decision + guard.** Decision rule: if the coverage
  model matches or beats the Task-2 bundle on philos **with fewer
  tuning knobs** (no `idle_halflife`, no wrong-math), prefer it. Run
  the autoferry guard before promoting to a canonical config.
- [ ] **Step 9 — docs.** Update the four sections in
  `docs/algorithms/pmbm-design.md`, and add/extend a plain-English
  chapter with a coverage/cadence diagram in `docs/learning/`
  (and `docs/learning/00-index.md` + glossary if it is a new chapter).

**Files touched:** `ports/ISensorActivity.hpp` (new);
`core/pmbm/PmbmTracker.{hpp,cpp}`; `core/types/Measurement.hpp` (id);
`adapters/benchmark/ReplayScenarioRun.cpp`; tests under `tests/pmbm/`;
docs as above.

---

## 11. Guardrails (from CLAUDE.md / the plan)

- **C++17 only.** Hexagonal: `core/` and `ports/` zero I/O; the philos
  activity model lives in `adapters/`.
- **Determinism test stays green.** No wall-clock, no RNG.
- **Back-compat:** the port is nullable and every new config defaults
  to today's behaviour; bare predict/update tests must be unchanged.
- **Autoferry guard:** any change promoted to a canonical config must
  not regress the existing PMBM autoferry win.
- **Prereq:** fix the known PMBM adaptive-birth **non-determinism**
  first (it adds noise to the single-seed philos numbers this spec is
  measured against).

---

## 12. What this retires when it lands

- the wrong-math `compute_miss_pD` crutch,
- the `idle_halflife_sec` fade-out hack,
- the `source_id="ais"` patch (Task 2a) — folded into the unified
  identity gate (`mmsi` else `platform_id`) + cadence model.

One honest coverage model in place of three shortcuts.

---

## 13. Roadmap (deferred, tracked here on purpose)

Items deliberately **out** of Task 4, kept so we do not lose them:

1. **Adaptive cadence provider (from decision §9a).** Replace the
   declared static profile with a model that *learns* each source's
   cadence/coverage from observed gaps, behind the same exchangeable
   interface introduced in Step 1. Lets cadence track reality (a vessel
   that speeds up reports more often; a radar whose rotation varies)
   without re-declaring config. No tracker changes — only a new
   implementation of the provider interface.
2. **Birth confidence by sensor kind (from decision §9b).** Timid
   surveillance births / confident cooperative births /
   most-confident Cooperative-channel births (see §7). Compounds with
   Task 1; pick up once the coverage model is in and stable.
3. **Target-dependent `p_D` (RCS / size).** Small vessels are genuinely
   harder to detect; today `p_D` is per-sensor, not per-target. A
   surveillance-side refinement so faint/intermittent small targets are
   not over-penalised on a miss.
