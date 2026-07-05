# Held-out results — sailboats_busy (scored against the locked pre-registration)

**What this is.** The held-out pass for `sailboats_busy`, scored against the
verbatim pre-registration in
[`2026-07-05_heldout_preregistration_sailboats_busy.md`](2026-07-05_heldout_preregistration_sailboats_busy.md)
— committed BEFORE the frozen detector touched the clip (ordering provable from
git). Frozen detector = `imm_cv_ct_pmbm_occupancy_detector` with
`membership_exit_factor = 0.6` (hysteresis ON, the deployable operator-facing
artifact); prediction 6's tracking is scored under `imm_cv_ct_pmbm_land` as the
pre-registration specifies. Evidence dumped by
`tests/replay/test_heldout_sailboats_probe.cpp`.

**Clip facts.** 120 s, 3221 radar plots (268 plots/10 s), 1365 radar scans, 0 AIS
rows / 0 MMSIs. Under the frozen detector: 1088 distinct confirmed track ids,
22 703 confirmed track-scans, **peak 4 simultaneous structure hazards**, ~51
hazard clusters over the clip (most transient: 3–15 scan-hits; a persistent line
at ~450 m). `extended_cells_min = 1`, so even a single persistent cell surfaces
as a hazard — every structure prediction is readable from the hazard map.

**Scoring:** hit / partial / miss, one sentence of evidence each.

| # | Prediction | Verdict | Evidence |
|---|---|---|---|
| 1 | Zero AIS in radar range | **HIT** | `ais.csv` empty, 0 MMSIs (the falsification clause — an in-range MMSI — did not occur). |
| 2 | Fleet builds ~no persistent cells; persistent mass is structure/moorings; high plot counts | **HIT** (plot-count sub-claim PARTIAL) | Peak only 4 simultaneous hazards over 1088 tracks; the persistent (high-scan-hit) mass is a shore line at ~450 m while the fleet appears only as brief flickers (3–15 scan-hits); plot rate 268/10 s (predicted 300–600 — same order, slightly low). |
| 3 | Far-bank shore group (42.357, −71.0837) reappears — KEEP_MIXED cross-validation from a different day | **PARTIAL** | Weak transient hazards sit right at the far-bank anchor (ENU ≈ (−350, −250), scan-hits 6–9), but the *strong* persistent line is at a nearby southern shore (42.354–42.355), not exactly the far-bank — the group cross-validates in direction but not as a strong persistent KEEP. |
| 4 | Longfellow Bridge pillars ~550–600 m ENE as compact persistent cells | **MISS** | No hazard cluster at the predicted 550–600 m ENE; the nearest NE structure is at ~350–450 m (a closer NE shore/dock, not the bridge). The pre-reg hedged this as possibly outside ENC coverage / detection range. |
| 5 | Esplanade/bank returns eaten by the land prior | **UNVERIFIABLE (partial)** | The coastline model is active (both configs), but "the land prior eats bank returns" is scorable only against bank *labels* or a no-land A/B — and this clip is held-out/unlabeled, so labeling it to score would break the held-out status. Deferred to a future labeled pass. |
| 6 | Churny fragmented dinghy tracks → high `false_unlabeled`, possibly the highest of any clip | **HIT** | 1088 distinct ids / 22 703 confirmed track-scans over 120 s = the **highest raw track mass of any clip** (vs close_approach 15 182, sunset 18 295) — exactly the intended demonstration that raw `gospa_false` is meaningless on philos. |
| 7 | Race-mark trap: bunching dinghies *might* bait the persistence classifier; predicted NOT to cross the bar on a short clip — falsification is the most valuable outcome | **PARTIAL — the valuable failure fired (mildly)** | The clip is **120 s, longer than the 20–80 s siblings the analyst assumed**, and the low-bar detector (`extended_cells_min=1`, `persistence_bar=0.2`) DID flicker transient hazards on the moving fleet in open water (scattered mid-basin clusters, 3–15 scan-hits) — the trap fired in transient form. But none matured into *sustained extended* open-water structure (peak 4 hazards; all high-persistence mass stays on shore lines). |
| 8 | One or two anchored committee/mark boats — compact persistent KEEP in open water | **PARTIAL** | Several moderately-persistent compact hazards (scan-hits 21–61) at mid-range in varied bearings are consistent with moored marks / a committee boat, but cannot be separated from shore points without the frames. |

**The big falsifier did NOT fire.** The pre-registration named the most damaging
outcome as *persistent EXTENDED structure mid-basin, away from banks/bridge/docks/
moorings* (would imply a projection/registration bug or unknown infrastructure).
All high-persistence mass is peripheral (shore lines); mid-basin is transient only.
So the analyst's model of philos and the clip's projection are intact.

**Freeze flip-guard (exit_factor 0.6 vs 1.0): zero flips — CHECKED, not assumed.**
Every prediction bets on WHERE structure appears (entry) or on content
(AIS/tracks); hysteresis changes only *exit* stickiness (how long a cell stays,
i.e. scan-hits), never the SET of locations that cross the bar. Verified
empirically by re-running the probe at 1.0 and diffing the hazard map on the
detector's 100 m cell grid: **126 of 128 occupied cells identical**; the sole
difference is one *transient* flicker landing in an adjacent SE cell (100,−400 vs
100,−500) — not near the far-bank, the bridge, or any prediction region, so no
item flips. Tracking (predictions 1, 6) is bit-identical (hysteresis touches only
the birth-suppression face, not the tracker). Scored against the frozen 0.6
artifact as specified.

## Tally and reading

**3 HIT (1, 2, 6), 1 MISS (4), 4 PARTIAL (3, 5, 7, 8).** The high-confidence bets
(zero AIS, fleet-doesn't-persist, highest false_unlabeled) all landed, and the
falsifier stayed quiet — the analyst's domain model generalized to an unseen clip.
The most *informative* result is prediction 7, exactly as pre-flagged: the frozen
detector's deliberately low bar transiently baits on the moving fleet in open
water — a real, mild instance of the race-mark trap — amplified because the clip
ran 120 s, longer than the assumed 20–80 s. That is a genuine, honestly-surfaced
weakness of the low-bar detector, not a suppression concern (nothing matured, and
per increment 8 the suppression is near-inert anyway), but a note for the
operator-facing hazard channel: transient open-water hazard flicker on dense
moving fleets is real, and hysteresis (now on) lengthens each flicker rather than
removing it. The clean MISS (bridge) and the unverifiable land-prior item (5) are
honest limits of a held-out, unlabeled clip.

`almost_cross` and `ais_ferry_far` carry no predictions (measured, not bet on);
their probe dumps are recorded for future tuning only.
