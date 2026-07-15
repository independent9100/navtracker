# W5 — Release-readiness findings (orchestrator-direct, 2026-07-11)

Run inline by the orchestrator (Opus 4.8) after the subagent quota was
exhausted. Each item traced with concrete grep/read evidence shown in the
review transcript; not independently verifier-checked, but doc-vs-code and
hygiene claims are directly reproducible from the commands noted.

## R1 — docs vs code

### RR-1 (HIGH, doc corroborates a confirmed code bug) — output-contract.md hard-codes the wrong covariance axis order
`docs/output-contract.md:8-9` states position_covariance_m2 is "target's local
NED frame (north-east; row/column 0 = north, row/column 1 = east)" and tells
consumers to read `σ_north = √(cov[0,0])`. The code (`TrackOutput.cpp:21`)
emits (east,north) order — no E/N permutation (bug B4 TrackOutput.cpp:21,
CONFIRMED). So the *doc actively instructs the wrong extraction*. The worked
example (line 47) labels an isotropic 25/25 matrix "(north, east)", which
cannot expose the swap. Also `app/example.cpp:144`, `integration-guide.md:123`,
and the Foxglove adapter follow the NED wording. Fix must touch code + all
doc sites together.

### RR-2 (MEDIUM/HIGH) — ADR-0002 "promote static→moving within bounded latency" clause has no implementing code or test
`docs/adr/0002-*.md:340-341`: "when a static starts moving, the system must
promote it to a vessel track within bounded latency (its returns leave the
occupancy footprint → they must [become a track])." Grep for any promotion /
de-anchor / static→moving path in `core/static/`, `StaticHazardEvaluator`,
`PmbmTracker` finds **no implementing code** (only an unrelated PMBM
re-promotion comment). The static-obstacle layer is a separate map that does
not suppress moving returns, so a re-mover likely *does* get born as a fresh
track — but there is no explicit bounded-latency guarantee and **no test**
asserts it (the test-sufficiency audit found none either). This is the
load-bearing half of the "presence over classification" invariant; it should
be demonstrated by a scenario test (anchored object begins moving → a moving
track appears within N seconds) before relying on ADR-0002 in the field.

### RR-3 (LOW, good news) — TODO/FIXME hygiene is excellent
Exactly **one** TODO in all shipping code: `AutoferryJsonReplay.hpp:52`
("enabling bearings requires a bearing-safe initiation path (TODO)") — which
is the same root cause as the RangeBearing2D-initiate bug (B1). No FIXME/HACK/
XXX/WORKAROUND anywhere in core/ports/adapters/app/sim/bench. Clean.

### RR-4 (LOW, good news) — learning docs are complete and index-consistent
29 chapters (`docs/learning/00..28`); index links all resolve; every major
algorithm has a chapter (PMBM ch23, coverage ch24, land-clutter ch25, static
obstacles ch26, occupancy ch27, bearing-wedge ch28, metrics ch20,
registration/bias ch21, alternatives ch22, plus the classic KF/EKF/UKF/PF/IMM/
JPDA/MHT/gating/NEES/CPA set). No missing-chapter or dangling-figure finding.

### RR-5 (LOW) — integration-guide config defaults spot-check passes
Sampled `CpaEvaluatorConfig.enter_probability` (0.5 ✓) and
`DatumRecenterPolicy.recenter_threshold_km` (30.0 ✓) — guide matches code.
The mechanical config-coverage test guards *mention*; these spot-checks
suggest field-level accuracy is also good. Minor: `rmc_stale_seconds` (adapter
config) not documented in the guide.

## R2 — API surface, ADR, README, hygiene

### RR-6 (MEDIUM/HIGH) — no install/export/package story for a library billed as consumable
`CMakeLists.txt` has **zero** `install()`, `export()`, or
`write_basic_package_config` rules. The integration guide tells a consumer to
link `navtracker_core`/`navtracker_nmea`, but there is no way to install the
library, no `find_package(navtracker)` config, no exported targets. A consumer
must vendor the source or `add_subdirectory`. For a pre-release of a
"hexagonal-architecture library" this is a real gap; at minimum document the
supported consumption path (add_subdirectory) and ideally add install/export
targets.

### RR-7 (LOW, good news) — layering invariant and port ABIs hold
`navtracker_core` links only `Eigen3::Eigen` (no nlohmann_json/mcap/I/O) — the
"core has zero I/O" invariant holds at the build-graph level. All 20 port
interfaces in `ports/*.hpp` declare a `virtual ~I…()` destructor (safe
polymorphic deletion). `navtracker_land` pulls nlohmann_json PRIVATE (correct).

### RR-8 (MEDIUM) — repo ships ~200 MB of baselines + copyrighted PDFs; a personal notes file is tracked
Total tracked tree ≈ 236 MB. Of that: `docs/baselines/` = **182 MB** of
metric-dump CSVs (incl. one 44 MB file), and `docs/references/` = ~21 MB of
academic PDFs — including `S0029801822005753-helgesen-2022-…pdf` (6.4 MB),
which is an **Elsevier publisher-copyrighted** article whose redistribution in
a shipped repo is a copyright concern (the arXiv PDFs are lower-risk).
`todo.md` at repo root is tracked but is a **personal KF/IMM learning-notes
scratch file**, not project docs — should not ship. Recommend: move baselines
to a release-excluded path or Git LFS, drop the Elsevier PDF (link/cite
instead), untrack `todo.md`. (`err.txt`/`next.md` are correctly untracked.)

### RR-9 (LOW) — charts/ENC zips are NOAA public-domain but carry a USERAGREEMENT.TXT
`charts/US5BOS*.zip` (784 KB total) are NOAA ENC S-57 cells (charts.noaa.gov),
US-government public domain — commercially usable. `charts/ENC_ROOT` includes a
`USERAGREEMENT.TXT`/`README.TXT`; confirm the no-warranty/attribution terms are
acceptable and keep them alongside the data. Low risk.

### RR-10 (LOW) — CLAUDE.md/README dependency list understates Conan requires
`conanfile.txt [requires]` = eigen 3.4.0, gtest 1.14.0, **mcap 1.4.1,
nlohmann_json 3.11.3**. CLAUDE.md's tech-stack section names only Eigen +
GoogleTest; README's build block doesn't enumerate deps. Minor doc drift; the
build itself is correct (README uses `cmake --preset conan-release`).

## Coverage notes
- Verified accurate: integration-guide config defaults (sampled), learning-doc
  index integrity, port virtual-destructor ABIs, core/ layering, TODO cleanliness.
- Not done inline (would benefit from a later verifier pass): exhaustive
  integration-guide field-by-field default diff (only sampled); readme code-
  snippet compile check; adr decision-table row-by-row.
