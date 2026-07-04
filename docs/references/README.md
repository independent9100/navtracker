# External references

Local copies of papers cited in navtracker's design docs, so the citation
survives even if the upstream link rots. Cite the arXiv/DOI in prose; keep the
PDF here for offline reading.

| File | Citation | Cited by |
|---|---|---|
| `2508.16169v1-herrmann-2025-hybrid-tbd-coastal-radar.pdf` | Herrmann, García-Fernández, Brekke & Eide, *A Scalable Hybrid Track-Before-Detect Tracking System: Application to Coastal Maritime Radar Surveillance*, arXiv:2508.16169v1 (22 Aug 2025; for IEEE J. Oceanic Eng.) | [ADR 0002](../adr/0002-static-objects-track-vessels-map-environment.md) "Prior art & validation"; design spec §14.10 (fixed-vs-moving caveat); `docs/algorithms/comparison-baselines.md` (TBD-for-weak-targets roadmap row) |
| `2502.18368v1-dalhaug-2025-near-shore-mapping-vessels.pdf` | Dalhaug, Stahl, Mester & Brekke, *Near-Shore Mapping for Detection and Tracking of Vessels*, arXiv:2502.18368v1 (25 Feb 2025), NTNU Trondheim | [ADR 0002](../adr/0002-static-objects-track-vessels-map-environment.md) "Prior art & validation"; design spec §14.10 (extent-is-interim discriminator); `docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md` R3 |
| `S0029801822005753-helgesen-2022-heterogeneous-multisensor-littoral.pdf` | Helgesen, Vasstein, Brekke & Stahl, *Heterogeneous multi-sensor tracking for an autonomous surface vehicle in a littoral environment*, Ocean Engineering 252 (2022) 111168 (PII S0029801822005753; **paywalled Elsevier**, dataset: autoferry.github.io/sensor_fusion_dataset) | `docs/baselines/helgesen2022_reference.md` (Cl-1 baseline + GOSPA methodology); `docs/algorithms/comparison-baselines.md` (source bibliography) |

**Why this one matters (summary):** the closest published system to
navtracker's static-obstacle direction — a real coastal X-band radar tracker
that runs a PMBM point tracker on DBSCAN-clustered detections, removes land with
a *separate* precomputed mask, tracks stationary non-AIS vessels, and pairs the
PMBM with an IE-PHPMHT **track-before-detect** channel for weak targets. It
validates our vessel-vs-environment split and PMBM-for-static-vessels choices;
the two deltas we recorded from it are (1) TBD for weak/low-SNR targets (a future
roadmap candidate — we are detection-only) and (2) their fixed-station median
land mask does **not** port to our moving platform, which is why our data-derived
static layer must be geo-referenced.

**Dalhaug 2025 — why it matters (summary):** the closest published system to
navtracker's Stage 1b/2 static-occupancy direction. It names *imprecise land
masking* as the central near-shore tracking failure — exactly navtracker's ADR
0001 no-birth-zone problem — and builds an offline LiDAR+camera map of the
docking area that deliberately **does not map potentially-moving objects**. Two
findings port directly to our review tickets: (1) it separates static structure
from movable vessels by **classification** (deep instance segmentation of the
camera image, projected onto the LiDAR points), *not* by geometry/extent —
independent support for R3's "extent is an interim discriminator" and for the
Stage-1b-ii corroboration plan (chart / AIS / camera). Their pipeline measured
both failure directions of a purely-learned classifier (a floating dock read as
two boats), which is why corroboration matters. (2) It states plainly that an
**ENC is only an *initialization*** for the map because ENC layers "are not
always as detailed as required for accurate target tracking near shore" — an
independent confirmation of our ~⅓ chart-coverage finding (eval-log 2026-07-01)
and of the decision that the real philos lever is a data-derived live-occupancy
layer, not the chart alone.
