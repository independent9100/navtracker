# PMBM `contributing_sources` — §14.11 (2026-07-15)

Branch `pmbm-contributing-sources` off master `66e55b3`. Origin: the F2 cycle's
Finding A — the deployable tracker (PMBM) left `TrackOutput.contributing_sources`
**empty** while the flat/MHT path populated it genuinely per-update. Arbiter
ruling: **POPULATE**. Design ruling and constraints in
`docs/superpowers/plans/2026-07-15-pmbm-contributing-sources-ticket.md`.

## What changed

PMBM now fills `Track::contributing_sources` from the F2-truthful claimed-source
channel, with the same semantics as flat/MHT (cumulative over the track's life,
deduplicated, first-seen order). Mechanism (four-part design note:
`pmbm-design.md` §3.6.1):

- A sibling map `bernoulli_sources_` (`BernoulliId → ordered unique source_ids`)
  is accumulated at the **same** site as `contribution_history_` — inside the
  claimed-measurement gate (`last_claimed_meas_index ≥ 0`), from
  `scan[claimed].source_id`, using the exact `find`-guarded-`push_back` idiom MHT
  uses for `tree_sources_`.
- Read in `refreshAggregatedTracks` into the aggregated output track's
  `contributing_sources` (keyed by the stable Bernoulli label id, so the
  aggregated track adopts the carrying Bernoulli's history across a hypothesis
  switch — last-write-wins, as with R11 `platform_id`).
- **Retention: cumulative** (NOT the 2 s window that bounds `recent_contributions`)
  — matches flat/MHT, so the field carries no per-tracker asterisk. Pruned only
  when the label leaves every hypothesis (same alive-id rule as the sibling maps).

Production diff: `core/pmbm/PmbmTracker.hpp` (one member) + `core/pmbm/PmbmTracker.cpp`
(accumulate at the claim site, prune on the alive-id sweep, read at output). ~15
lines. No flag — unconditional, matching flat/MHT.

## Attributes-only — R3 two-class A/B (byte-identical metrics)

ON (branch) vs OFF (master `PmbmTracker` swapped in, same bench binary),
`--config-filter pmbm --skip-replays --seeds 1`: **18 PMBM configs × 26 scenarios**,
24,734 metric rows. After excluding wall-clock timing (`wall_seconds`,
`scan_proc_ms_*` — inherently non-deterministic), the **21,468 substantive metric
rows are byte-identical**: OSPA, GOSPA, T-GOSPA, `card_err_mean`, NEES (mean/median/
p95/p99/coverage), NIS, pos/sog/cog RMSE, `id_switches`, `lifetime_ratio`,
`track_breaks`, occupancy metrics, and every per-truth breakdown. Kinematics /
existence / lifecycle / cardinality untouched — `bernoulli_sources_` feeds only the
output attribute; no gate, likelihood, existence, or lifecycle path reads it.

## Invariant + tests (teeth per #24)

- `tests/pmbm/test_pmbm_contribution_provenance.cpp` — the F2 invariant extended to
  the **output** field: `OutputContributingSourcesExcludesForeignSource` (a
  radar-only track lists `radar`, never the foreign `ais`) and
  `OutputContributingSourcesUnionsGenuineContributorsCumulatively` (alternating
  radar/AIS on one Bernoulli → both listed, cumulative after later radar-only scans,
  exactly two, deterministic first-seen order).
- `tests/integration/test_t2t_live_pedigree_content.cpp` — the T2T E2E pedigree pin
  extended: `PmbmBackedSourcePedigreeIsGenuine` drives a PMBM tracker through
  `NavtrackerSource::toExternalTrack` and asserts a two-sensor track shows both
  `Used`, a radar-only track leaves `ais` non-`Used`. Completes the pedigree story
  the F2 Rider-B lift started (a PMBM-backed source now yields genuine `Used`
  pedigree instead of all-`Unknown`).
- **Teeth (mutation → RED):** M1 (disable the output fill) → the population
  assertions go RED across all three new tests; M2 (inject a spurious foreign
  source) → the invariant assertions go RED. Both reverted; final state 6/6 GREEN.

## Consumer surface

- `docs/output-contract.md` — `contributing_sources` note: all three trackers
  populate it, same semantics.
- `docs/integration-guide.md` §3.10 — the pedigree note rewritten (was "PMBM leaves
  it empty → all-Unknown"); PMBM now genuine, diagnostics-only reaffirmed.
- `docs/superpowers/specs/2026-05-28-…-design.md` §14.11 — marked RESOLVED (POPULATE).
- `docs/algorithms/pmbm-design.md` §3.6.1 — four-part design note.

## Ways to improve

Per-source last-contributed timestamp (distinguish currently-radar-only from
AIS-touched-long-ago); carry `SensorKind` alongside `source_id`; evaluate a
bounded-history variant for very long-lived tracks. None change the cumulative
default. See `pmbm-design.md` §3.6.1.
