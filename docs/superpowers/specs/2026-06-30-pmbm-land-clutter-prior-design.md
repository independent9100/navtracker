# PMBM land / coastline clutter-prior — design spec (Task A)

Status: design, approved for spec 2026-06-30. Not yet implemented.
Author dialogue: brainstorming session 2026-06-30 (decisions recorded in §9).

This is the principled fix for the **philos near-shore-clutter over-count**,
now empirically confirmed: on the philos replay (Boston inner harbor) the PMBM
over-count (card_err ≈ +108) is **185 persistent, stationary radar returns at
fixed shore positions** that no AIS vessel ever occupies — i.e. shoreline /
pier / structure clutter, born as phantom tracks. (Pre-check 2026-06-29:
azimuth calibrated to 0.23° RMSE; 185 fixed clutter clusters > 75 m from any
vessel ≈ the observed over-count.) The Task-4 coverage/visibility channel
cannot fix this — at philos radar p_D = 0.07 a missed sweep is near-zero
evidence and shore returns are *re-detected every rotation*, so the problem is
**spatial, not temporal**. This spec adds the missing spatial lever: suppress
phantom *births* on land, using a coastline the consumer supplies as GeoJSON.

---

## 1. The one idea

The consumer provides **GeoJSON of the surroundings** (land/coastline polygons,
geodetic lat/lon, a reasonable radius around own-ship). From it we compute a
continuous **clutter prior** at any position: ~0 over open water, →1 on/at the
shore. At new-target **birth**, that prior **down-weights (and, on land,
refuses)** the new track — killing shore-return phantoms at the source while
leaving real open-water targets untouched.

---

## 2. The port (core, pure, nullable)

```cpp
// ports/ILandModel.hpp
#pragma once
#include <Eigen/Core>
namespace navtracker {

// Continuous spatial clutter/land prior, queried by the tracker at birth time.
// Pure, zero-I/O: a function of (current coastline geometry, current datum).
// Nullable in use — if no land model is wired, behaviour is exactly today's.
class ILandModel {
 public:
  virtual ~ILandModel() = default;

  // Prior that a detection at this ENU position is shore/structure clutter
  // rather than a real new vessel.
  //   0.0  = open water        (no birth suppression)
  //   →1.0 = on / at the shore (strong birth suppression)
  // Positions outside the loaded coastline's coverage return 0.0 (unknown →
  // do no harm), which is what gives graceful "stale until refreshed" after a
  // datum recenter (see §4).
  virtual double clutterPrior(const Eigen::Vector2d& enu_xy) const = 0;
};

}  // namespace navtracker
```

The core only ever calls `clutterPrior`. GeoJSON parsing / fetching is I/O and
lives in an **adapter** (the concrete implementation), preserving the hexagonal
boundary (invariant 1).

---

## 3. Birth integration (the suppression mechanism)

**Why not λ_C.** The obvious route — "raise the clutter intensity λ_C near
shore via the detection model" — is **silently defeated** by Task 1's
`birth_existence_target`. With adaptive birth and a target r*, the code sets
`λ_birth = (r*/(1−r*))·λ_C`, so

```
r_new = λ_birth / (λ_birth + λ_C) = r*        (independent of λ_C — verified)
```

Raising λ_C does nothing to r_new under the very birth knob that wins philos.
So the land prior must act on the **birth existence directly**.

**The rule (decision §9a — soft + hard gate).** For each new-target candidate,
born at its measurement's ENU position `p`:

```
const double c = land_model_->clutterPrior(p);   // 0..1
if (c > cfg_.land_birth_hard_gate)   skip this birth candidate entirely;
else                                 r_new *= (1.0 - c);
```

- **Soft** (`r_new *= 1−c`): suppression scales with the prior. The protection
  for real near-shore targets comes from the **shape of the prior** (§6): it is
  a *signed ramp centred on the shoreline* — ≈0.5 at the waterline, rising to a
  plateau of 1.0 only **well inside land**. So a vessel anchored right at the
  shore sits in the mid-ramp (`c≈0.5`, `r_new` merely halved) and, if
  persistently detected, can still confirm a track.
- **Hard gate** (`c > threshold` → no birth, threshold e.g. 0.95): because `c`
  only reaches the plateau **well inland**, the hard gate fires *only* for
  points clearly on land/structures (buildings, pier interiors) — never at the
  waterline. This decisively removes the on-land phantoms (a pure soft term
  leaves a birth-mass sliver that ramps a phantom over a long replay) while
  **never hard-killing a water position**.

**Anti-circularity / anchored-ship safety (refined 2026-06-30):** the hard gate
is deliberately *inland-only* (via the ramp shape), so we never hard-refuse a
birth where a real anchored/moored non-AIS vessel could physically be (on
water, at the waterline). The waterline strip is soft-only. We do NOT use a
"re-detected every scan ⇒ override the gate" rule — shore returns are *also*
re-detected every scan, so that would reopen the very philos hole we are
closing.

Applied **only** when a land model is wired AND adaptive birth is producing
candidates; with no land model, the birth path is bit-identical to today.

**Birth position.** Position2D measurements carry ENU directly; range/bearing
measurements use the same ENU projection the birth candidate already uses. The
prior is queried at that birth position — no new geometry needed in the hot
path beyond the `clutterPrior` call.

---

## 4. GeoJSON + datum recenter — store geometry in geodetic

The working ENU datum **auto-recenters** when own-ship moves > 30 km
(`OwnShipProvider`, fires `IDatumChangeSink::onDatumRecentered(old,new)`
synchronously inside `update(pose)` — i.e. inside the deterministic, timestamp-
ordered flow). The coastline must survive that.

**Design: store the GeoJSON polygons in their native geodetic (lat/lon)
frame — never in ENU.** Then:

- **Query** `clutterPrior(enu)`: `geo = datum.toGeodetic(enu)`; evaluate the
  geodetic polygons at `geo`. (`Datum::toGeodetic` already exists.)
- **Datum recenter is trivial:** the land model registers as an
  `IDatumChangeSink`; on `onDatumRecentered(old,new)` it simply swaps the datum
  it uses for the ENU→geodetic query conversion. **No polygon reprojection, no
  cache rebuild** — the polygons are datum-independent.
- **"Serve stale until fresh" falls out for free:** after a > 30 km recenter
  the old GeoJSON no longer *covers* the new area, so `clutterPrior` returns
  0.0 there (no suppression, never invented land). Everything the old coastline
  *does* cover keeps working. We continue on the outdated coastline and degrade
  safely until fresh GeoJSON arrives — exactly the required behaviour.

---

## 5. Async update + determinism

A second adapter-side method swaps the coastline when fresh data arrives:

```cpp
// On the concrete adapter (NOT on the core ILandModel query interface):
void setCoastline(<parsed geodetic polygons for the new area>);
```

**Determinism contract (CLAUDE.md invariant 4).** The land model itself is a
**pure function** of (current polygons, current datum) — no wall-clock, no RNG,
no threads. The *fetch* is async and lives entirely in the consumer/adapter;
the *swap* must be applied at a **deterministic point in the timestamp-ordered
stream**. In replay, a coastline update is a logged, timestamped event applied
in order — same input ordering → identical output. The spec mandates: never
mutate the active coastline from a wall-clock callback mid-scan; stage the new
coastline and apply it at a scan boundary tied to a timestamp.

---

## 6. Behind the port (implementation detail, not interface)

- **Continuous prior from polygons — a signed ramp centred on the shoreline.**
  Let `d` be the signed distance to the nearest shore edge: `d < 0` inside land
  (|d| = how far inland), `d > 0` offshore. Then
  `c(d) = clamp( (W_off − d) / (W_off + W_in), 0, 1 )`, i.e.:
  `c = 1.0` for `d ≤ −W_in` (well inland → plateau, the hard-gate region);
  a smooth `1→0` ramp across the band `[−W_in, +W_off]` (so `c ≈ 0.5` at the
  waterline `d=0` when `W_in≈W_off`); `c = 0.0` for `d ≥ +W_off` (open water).
  `W_in` (inland half-width) and `W_off` (offshore half-width) are configurable;
  the ramp is what protects anchored/near-shore real targets (§3) and absorbs
  the coarse, administrative-grade waterline imprecision of the GeoJSON.
- **Hot path.** `clutterPrior` is called per birth candidate per scan.
  Rasterize the GeoJSON into a local grid (clutter-prior field) once per
  coastline snapshot for O(1) lookup; rebuild only when `setCoastline` swaps in
  fresh data. Deterministic; bounded memory.
- **GeoJSON parsing.** Accept Polygon / MultiPolygon land features. A tiny
  dependency-free parser for the subset we need, or a vetted header-only JSON
  lib added via Conan (noted per CLAUDE.md if introduced). Parser validates at
  the edge (invariant 6).

---

## 7. Wiring (composition root)

```cpp
// adapter implementing both the query port and the recenter sink:
class GeoJsonLandModel : public ILandModel, public IDatumChangeSink { ... };

GeoJsonLandModel land{...};                 // built from initial GeoJSON
provider.registerDatumSink(&land);          // learns recenters (existing hook)
pmbm.setLandModel(&land);                    // new nullable setter on PmbmTracker
// async: land.setCoastline(...) applied at a deterministic stream point
```

`PmbmTracker::setLandModel(const ILandModel*)` mirrors the existing nullable
setters (`setSensorActivity`, `setSensorDetectionModel`). Null = today's
behaviour, bit-identical.

---

## 8. Required four-section algorithm doc (CLAUDE.md standard)

**Math.** Birth existence with a spatial clutter prior `c = clutterPrior(p) ∈
[0,1]`: `r_new⁺ = r_new · (1 − c)` for `c ≤ gate`, and the candidate is dropped
(no birth) for `c > gate` (gate e.g. 0.95). `c` is a *signed ramp centred on the
shoreline*: with `d` = signed distance to nearest shore edge (`d<0` inland),
`c(d) = clamp((W_off − d)/(W_off + W_in), 0, 1)` → `c=1` for `d ≤ −W_in` (well
inland), smooth `1→0` across `[−W_in, +W_off]` (`c≈0.5` at the waterline),
`c=0` for `d ≥ +W_off`. Because `c` reaches the gate threshold only on the
inland plateau, the hard gate fires inland-only and never at the waterline.
Positions outside coastline coverage → `c = 0`. Acts on birth existence, NOT on
λ_C (which is decoupled from `r_new` by `birth_existence_target`).

**Assumptions.** (1) The consumer supplies coastline GeoJSON covering a
reasonable radius around own-ship, in WGS84 lat/lon. (2) The coastline is
approximately correct at the margin scale (tide/resolution handled by the soft
band, not a hard edge). (3) Datum recenters are observed via `IDatumChangeSink`;
coastline updates are applied at deterministic stream points. (4) Real targets
that genuinely sit on the rasterized "land" (e.g. a vessel alongside a pier
finer than the grid) are rare and acceptable to suppress at birth — they
re-confirm if persistently detected (soft term) unless past the hard gate.

**Rationale.** The philos over-count is empirically *spatial* clutter (185 fixed
shore returns), which a temporal coverage model cannot remove (low p_D, returns
re-detected each sweep). Suppressing births by a coastline prior attacks the
phantom at its source. Injecting at birth existence (not λ_C) is required
because Task 1 decoupled `r_new` from λ_C. Storing geometry in geodetic makes
the auto-recenter handling trivial and the stale-until-fresh behaviour safe by
construction.

**Ways to improve / what to test next.** Roadmap (§10): coverage-occlusion
(land between sensor and track ⇒ don't charge a surveillance miss — couples with
Task 4); on-land plausibility gating for *existing* tracks that drift ashore;
learning the clutter field online (cf. the existing `ClutterMapDetectionModel`)
to complement the static coastline; sourcing/validating real coastline GeoJSON
for benchmark areas.

---

## 9. Decisions (settled 2026-06-30)

- **(a) Birth suppression = SOFT + INLAND-ONLY HARD GATE.** `r_new *= (1 −
  clutterPrior)` always; drop the candidate if `clutterPrior >
  land_birth_hard_gate` (e.g. 0.95). The prior is a signed ramp centred on the
  shoreline (≈0.5 at the waterline, plateau 1.0 only well inland, §6), so the
  hard gate fires **inland-only** — never at a water position where a real
  anchored/moored vessel could be. Refined 2026-06-30 after the "is it clutter
  or anchored ships?" review. See §3.
- **(b) First-cut scope = BIRTHS ONLY.** The proven philos lever. Coverage-
  occlusion and on-land plausibility gating are deferred to the roadmap (§10).
- **(c) Port contract = continuous `clutterPrior(enu) → double`** computed by
  the adapter from consumer-supplied GeoJSON. (Binary land/water is the extreme
  case.)
- **(d) Geometry stored in GEODETIC**, query-time ENU→geodetic conversion;
  datum recenter swaps the query datum only; out-of-coverage → 0.0
  (stale-until-fresh). See §4.
- **(e) Determinism:** land model is a pure query; async coastline swap applied
  at a deterministic timestamp-ordered point, never a wall-clock callback. §5.

---

## 10. Implementation outline (TDD; one commit per step group)

Reuse the shared build/bench protocol from prior plans. Determinism test stays
green; new port nullable + default-off bit-identical; autoferry guard before
promoting any canonical config.

- [ ] **Step 1 — the port.** `ports/ILandModel.hpp` (pure interface, documented).
- [ ] **Step 2 — clutter-prior geometry.** A geodetic polygon set + signed-
  distance-to-shore with margin falloff; rasterized grid for O(1) query.
  Unit-test prior values: inside land ≈1, mid-band linear, open water 0,
  out-of-coverage 0.
- [ ] **Step 3 — GeoJSON adapter.** Parse Polygon/MultiPolygon (validate at
  edge) → the geometry of Step 2. `GeoJsonLandModel : ILandModel,
  IDatumChangeSink`. Unit-test parsing + that `onDatumRecentered` swaps the
  query datum (a fixed lat/lon point keeps its prior across a datum change;
  a point only in the *old* area returns 0 after recenter to a far new area).
- [ ] **Step 4 — async swap + determinism.** `setCoastline(...)` replaces the
  snapshot; test bit-identical replay when the swap is applied at the same
  stream point twice; test no wall-clock/RNG.
- [ ] **Step 5 — PMBM birth integration.** `PmbmTracker::setLandModel(const
  ILandModel*)` + Config knobs `bool use_land_model=false`,
  `double land_birth_hard_gate` (default conservative). Apply soft+hard gate at
  the new-target candidate. TDD: a birth at a high-prior position is dropped; a
  mid-band birth has reduced r_new; an open-water birth is unchanged; null land
  model = bit-identical.
- [ ] **Step 6 — philos validation.** Obtain coastline GeoJSON for the philos
  area (see §11 risk); new bench config `imm_cv_ct_pmbm_coverage_land` (coverage
  + land model). A/B vs bundle/birthtarget; success = card_err / gospa_false
  collapse toward the birthtarget baseline or better. Autoferry guard.
- [ ] **Step 7 — docs.** Update the four-section doc and add/extend a plain-
  English `docs/learning/` chapter (coastline → clutter prior → birth gate, with
  a diagram); cross-link.

**Roadmap (deferred):** coverage-occlusion (couples with Task 4); on-land
plausibility gating for existing tracks; online clutter-field learning;
real-coastline sourcing tooling.

---

## 11. Risks / open dependencies

- **Coastline GeoJSON for philos — RESOLVED 2026-06-30.** A real GeoJSON was
  obtained: `boston.geojson` ("City of Boston Boundary (Water Included)", 18
  Polygon features, CRS84 lon/lat, bbox lon −71.19..−70.87 / lat 42.23..42.40).
  Validated empirically: land points (downtown, Charlestown Navy Yard) test
  inside; water points (own-ship, mid-harbor, far vessel) test outside; and
  **86.2% of philos radar plots fall on-land (69.2%) or within 50 m of shore
  (17.0%)**, vs 13.8% open-water — i.e. the mask catches exactly the clutter we
  suppress at birth, leaving open-water (real-vessel) births alone. Piers are
  resolved. So A is validated on the **real** philos replay (A → E order kept).
  Move the file into `tests/fixtures/philos/`. NOTE: despite its name it behaves
  as a land mask for our points; the soft margin band covers the
  administrative-boundary (non-survey-grade) waterline imprecision. We did NOT
  derive the mask from the clutter (would be circular) — it is independent
  municipal data.
- **GeoJSON dependency.** If a JSON lib is needed, add via Conan and note it
  (CLAUDE.md). Prefer a minimal parser for the Polygon subset to avoid a heavy
  dependency.
- **Grid resolution vs shoreline detail.** Too coarse → suppresses near-shore
  real boats; too fine → memory/time. Make cell size configurable; default
  ~25 m (matches the clutter-cluster scale found in the pre-check).
- **"Clutter vs anchored non-AIS ship" ambiguity.** A persistent stationary
  no-AIS return can be a fixed structure, an anchored vessel, or sea clutter;
  the data cannot separate them per-return. Evidence it is mostly structures:
  69% of philos returns are physically *on* land (a ship cannot be), only 17%
  are in-water within 50 m. Mitigation (decision §9a): the hard gate is
  inland-only and the waterline is soft (ramp `c≈0.5`), so an anchored vessel at
  the shore is merely down-weighted and can still confirm — we never hard-kill a
  water position. Residual risk: a non-AIS vessel moored *against* the shore,
  inside the coarse polygon past `W_in`, could be hard-gated; accepted as small
  and bounded, revisit with finer charts / target classification if it bites.
  For the benchmark it is moot (truth is AIS-only → both score as over-count).
```
