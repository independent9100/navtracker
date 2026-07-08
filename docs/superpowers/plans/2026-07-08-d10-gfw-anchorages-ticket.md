# Implementer prompt — D10: Global Fishing Watch anchorages — license gate, extract, and how much of D3 it kills

Status: ready to hand off. Paste everything below the line. Origin: data-
expansion item D10 (`docs/superpowers/plans/2026-07-02-data-expansion-todos.md`
§D10), pre-water selection doc item 13. North-star tag: Cl-3 corroboration
seam (KEEP-side chart prior) + D3 replacement. Mostly Python against
downloaded data; use a worktree for anything committed
(`git worktree add ../navtracker-d10 -b d10-gfw-anchorages`). Budget
~half day to 1 day.

---

## Step 0 — LICENSE GATE (before downloading anything)

Global Fishing Watch data ships under its own terms (historically CC BY-NC
variants + site terms). We are a commercial context. The D7 MOANA precedent
applies: CC-BY-NC-SA was a hard NO-GO there. Read the current license/terms
for the anchorage dataset and the events API, quote the operative clauses in
your report, and classify: (a) usable as a shipped fixture/prior,
(b) usable ONLY as research-side statistics that inform our own design
(numbers derived by us, data never shipped), or (c) not usable. If (c),
STOP and report. If (b), scope everything below to research-side use and
say so in every artifact. Do not let the interesting data argue you past
the license.

## Step 1 — register + pull the anchorage layer

Registration is free. Pull the anchorage database and extract the two bench
geographies: Boston harbor area (philos) and the HAXR site. Data goes under
`data/gfw/` in the MAIN tree (gitignored — big data never enters git);
record checksums + exact query/version in the eval-log entry.

## Step 2 — sanity anchor: the video-verified cluster

Check that the Boston ~42.3585 N anchorage (our video-verified KEEP cluster
from the R8.8/occupancy work) appears in the GFW layer. This is the one
ground-truth point we own. Report hit/miss with the nearest GFW anchorage
polygon/point and its distance. A miss is a finding about GFW's coverage
threshold (≥20 unique vessels since 2012), not a failure.

## Step 3 — the two assessments (the deliverable)

1. **D3 replacement.** D3 wanted anchored-vessel dwell / transition /
   watch-circle statistics (for the AIS-veto and anchored-vessel
   corroboration design). How much of that is pre-computed in GFW's
   anchorage + event data? State per-statistic: covered / derivable /
   still needs raw MarineCadastre mining. The output is a shorter D3.
2. **KEEP-side corroboration prior.** The anchorage polygons as a chart
   prior: "this water is a known anchorage → bias KEEP" — the opposite
   polarity of the charted-structure SUPPRESS prior, feeding the same
   corroboration seam. Assess: polygon quality/size at our geographies,
   false-positive risk (do GFW anchorages cover fairways?), and what the
   wiring would be (a note, not an implementation).

## Acceptance

1. License classification with quoted clauses (Step 0) — this is the
   FIRST section of the report.
2. Extracts for both geographies under `data/gfw/` (gitignored) with
   checksums; committed deliverable = report doc
   `docs/baselines/2026-07-08_d10_gfw_anchorages.md` + dated eval-log
   entry. No GFW data committed to git.
3. Boston sanity-anchor result stated (Step 2).
4. Per-statistic D3-coverage table + prior assessment note (Step 3);
   no code changes, no prior wired.
5. Stop-and-report: license is (c); registration wall requires
   organizational credentials you don't have; or the anchorage layer has
   no data at either bench geography.
