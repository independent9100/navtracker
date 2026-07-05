# Held-out pre-registration — sailboats_busy (locked)

**Provenance.** These predictions were registered conversationally on
2026-07-03, BEFORE the held-out clips' CSVs or frames were opened by anyone,
and extracted VERBATIM from the session transcript on 2026-07-05 (before the
frozen detector touched any held-out clip). They are committed here so the
held-out pass scores against a locked artifact, not a memory.

**Scope.** Predictions cover `sailboats_busy` only. `almost_cross` and
`ais_ferry_far` carry no pre-registration — they are measured without
predictions (still held-out for tuning purposes, but only sailboats_busy
tests the analyst's model of the domain).

**Scoring protocol.** Run the FROZEN detector config first, then label/inspect,
then score each numbered item hit / partial / miss with one sentence of
evidence. Items 7 and 8 are the highest-information bets in either direction.
Prediction 1's falsification clause counts as specified in the text.

---

Good exercise — pre-registering predictions before looking is exactly the discipline that keeps the held-out set honest. Here's what I expect, using only what's already known (the clip's own-ship bounding box from the earlier proximity mapping, the R4 cell statistics, and the geography) — I haven't opened its CSVs beyond that, and no frames:

**Setting.** Own-ship loiters in a ~130 m box in the Charles River basin (~42.358, −71.080), east of the `close_approach` area, probably the same late-October 2021 recording campaign. Slow or station-keeping own-ship, like the other basin clips.

**High confidence:**
1. **Zero AIS in radar range.** The basin is dinghy water; nothing there carries a transponder. `ais.csv` empty or nothing within ~1 km. (Falsified if: any in-range MMSI — would actually be a useful surprise, a first AIS-corroboration test case in the basin.)
2. **The name is the content: a large fleet of small sailing dinghies, mostly moving.** Radar-wise that means high plot counts (I'd guess the same order as `close_approach`, ~300–600 per 10 s), a large number of *distinct* world cells, and — critically — **the dinghies build almost no persistent cells**, because sailing boats are world-transient. The persistent mass will be structure and moorings, not the fleet.
3. **The far-bank shore group (42.357, −71.0837) shows up again** — it's an `nscen=3` region and this clip is one of the three. Its returns should sit at roughly 200–350 m from own-ship. Under the KEEP_MIXED label from `close_approach`, this clip becomes its first cross-validation from a different day.

**Medium confidence:**
4. **Longfellow Bridge pillars enter the picture.** From that own-ship position the bridge is ~550–600 m ENE — inside radar range. I expect a line of compact persistent cells there: small-extent structure, exactly the "median extent = 1 cell" class from R4. Whether they're chart-matched depends on ENC coverage of the basin, which I'm not sure of — so they may land in the UNKNOWN/outside-coverage bucket.
5. **Esplanade/bank returns on both sides**, mostly within the coastline model's suppression reach — the land prior should eat most of them in the label-scored replay.
6. **Tracking behavior under `imm_cv_ct_pmbm_land`:** lots of short, churny tracks on the dinghies — they tack and jibe (the CT mode should help), they're small and intermittent, so I expect fragmented lifetimes and a high `false_unlabeled` in the decomposition (dozens of real, moving, unlabeled boats scored against an AIS truth that sees nothing). If we ever wanted a single clip to demonstrate *why* raw gospa_false is meaningless on philos, I predict this will be it — possibly the highest false_unlabeled of any clip.

**Lower confidence / genuinely uncertain — the interesting bets:**
7. **A race-mark cluster trap.** If the fleet is racing, dinghies bunch at marks and the start line. Over a short clip that *might* produce a few medium-dwell cells in open water — the one scenario where moving boats could bait a persistence classifier. If the clip is only 20–80 s like its siblings, I expect this *not* to cross any classification bar — but if I'm wrong, it's the most valuable failure mode the held-out set could surface.
8. **One or two anchored/committee boats.** Regattas often have a stationary committee boat or moored marks — compact, genuinely persistent, in open water. If present, they're the basin's version of the `sunset_cruise` ferry: compact persistent KEEP objects that dwell statistics cannot distinguish from a pile.

**What would most falsify my model of philos:** persistent *extended* structure appearing mid-basin away from banks, bridge, docks, or moorings — nothing in the geography explains that, so if it shows up, either the projection/registration has a problem in this clip or there's uncharted infrastructure we don't know about.

That's the pre-registration. When the detector and decay work land and we do the held-out pass, we check these eight against the frames — items 7 and 8 are the ones where being wrong teaches the most.
