# External references

Local copies of papers cited in navtracker's design docs, so the citation
survives even if the upstream link rots. Cite the arXiv/DOI in prose; keep the
PDF here for offline reading.

| File | Citation | Cited by |
|---|---|---|
| `2508.16169v1-herrmann-2025-hybrid-tbd-coastal-radar.pdf` | Herrmann, García-Fernández, Brekke & Eide, *A Scalable Hybrid Track-Before-Detect Tracking System: Application to Coastal Maritime Radar Surveillance*, arXiv:2508.16169v1 (22 Aug 2025; for IEEE J. Oceanic Eng.) | [ADR 0002](../adr/0002-static-objects-track-vessels-map-environment.md) "Prior art & validation"; design spec §14.10 (fixed-vs-moving caveat); `docs/algorithms/comparison-baselines.md` (TBD-for-weak-targets roadmap row) |

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
