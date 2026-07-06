# Implementer prompt — perf round 2: fresh profile + low-hanging fruit (post-Murty-fix)

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: 2026-07-06 — the Murty K=1 early exit (45a504d) removed the 85%
bucket and made the old profile obsolete. Raw density now runs 2.0× faster than
realtime but FAILS the ≥5× margin gate (141.4 s vs the ≤57 s the gate demands
on the 285 s window). This ticket hunts remaining cheap wins and closes the
per-scan-latency measurement gap. Unlike the first probe, implementing fixes
IS in scope — but only the safe class (see rules).

---

You are working in the navtracker repo (C++17, CMake+Conan, read `CLAUDE.md`
first). Mission: re-profile PMBM now that the Murty fix landed, implement any
LOW-RISK wins the fresh profile exposes, and add per-scan worst-case latency
measurement. Three hard rules:

- **Profile first, fix second.** The 2026-07-05 profile is OBSOLETE — its 85%
  bucket no longer exists, and gprof demonstrably over-attributed at least one
  bucket (it claimed the bench's own GOSPA scoring was ~10.5% of 515 s ≈ 54 s;
  the entire post-fix run is 41.6 s, so that number was instrument distortion).
  Nothing is a hotspot until the NEW profile says so.
- **Only the safe fix class, TDD'd.** Eligible: changes that are
  output-identical by construction (like the Murty early exit — prove it via
  the full suite + a byte-compare of bench metrics on the standard workload)
  or pure harness/bench changes that don't touch tracker output. NOT eligible
  without arbiter sign-off: anything that changes tracker results, however
  slightly — report those as priced findings instead. One commit per fix, each
  with its own before/after wall number.
- **Every claim gets a number from THIS round.** No reasoning from the old
  frontier doc's percentages.

## Setup

1. Worktree isolation again (parallel sessions are active):
   `git worktree add ../navtracker-perf2 -b perf-round-2` off current master.
   Own build dir. Conan-cache sandbox gotcha applies (see CLAUDE.md build notes).
2. Profiler: try `perf` first — it needs `kernel.perf_event_paranoid` ≤ 2
   (currently 4). Ask the user to run `! sudo sysctl kernel.perf_event_paranoid=1`
   in-session before falling back to gprof. If you must use gprof, build a
   SEPARATE `-pg` binary for profiling only and take all wall numbers from the
   clean (non-instrumented) Release build — never quote instrumented wall time.
3. Quiet machine for wall numbers; discard one warm-up run per batch
   (CPU-frequency drift is documented: first runs +20%).

## Workloads (both; raw is the one that matters now)

- **Decimated**: `tests/fixtures/haxr_cfar/out/kattwyk_08_dec50_w285.csv`
  (md5 `304cdeb8e81f03cbddb52d629fab22a9`), baseline **41.6 s** wall.
- **Raw-density**: cut from `kattwyk_08_full.csv` by tod ∈
  [29096.383, 29380.922] (299 981 rows; recipe in the 2026-07-06 eval-log
  entry — regenerate, record md5 of your cut). Baseline **141.4 s** wall.
- Both: `navtracker_bench_baseline --with-haxr --scenario-eq haxr --config-eq
  imm_cv_ct_pmbm_coverage_land`, `HAXR_PLOTS_CSV` pointing at the fixture.
- Baseline metrics that must stay byte-identical after safe fixes —
  decimated: gospa 104.262 / card_err 48.7626 / lifetime 0.104497 / id 0;
  raw: gospa 143.351 / card_err 100.528 / lifetime 0.186486 / id 0.

## Step 1 — per-scan latency instrumentation (do this FIRST, it's a deliverable)

The standing gap: all realtime numbers are replay MEANS; live operation cares
about the worst scan. Add per-scan processing-time capture to the bench path
(additive, default-on is fine if overhead is unmeasurable; it's harness-side):
report **mean / p95 / p99 / max scan time** and the scan interval, per run, as
bench columns. Acceptance: on both workloads, a table of those columns +
whether **max** scan time fits inside one scan interval (the honest realtime
statement). This instrumentation is also your measurement tool for Step 3.

## Step 2 — fresh profile (both workloads)

Bucket the top cost as before (cost-matrix construction / assignment /
hypothesis bookkeeping / occupancy / memory-allocation churn / bench harness /
other). Candidates the arithmetic already points at — treat as hypotheses:
- **Remaining Hungarian solves**: one seed solve per murtyKBest call survives
  the early exit; the harness GOSPA scoring also calls hungarianAssignment.
- **Cost-matrix construction**: ~15–25k KF/IMM predicted-measurement +
  likelihood evaluations per scan; Eigen temporaries.
- **Dense LSAP on gated matrices**: the solver factorises +∞ cells a
  sparse/gated variant would skip (the probe's named follow-up (a)).
- **Bench harness tax**: measure it honestly this time (time with/without
  scoring, not via gprof attribution).
Deliverable: bucket table per workload + one paragraph naming the top lever
for the RAW workload specifically.

## Step 3 — implement safe wins

For each (in profile-priority order, only if the profile confirms it matters):
implement → full suite green → byte-compare bench metrics on BOTH workloads →
record before/after wall + per-scan p99/max. Stop when the next candidate is
no longer safe-class or buys <5%. If a result-changing candidate looks big
(e.g. tighter gating), do NOT implement — write it up with its projected win
as a decision for the arbiter.

## Acceptance

1. Per-scan latency columns shipped + the max-vs-scan-interval table for both
   workloads (pre- and post-fixes).
2. Fresh profile bucket tables (both workloads) + named top lever for raw.
3. Each implemented fix: own commit, before/after wall, byte-identical metrics
   proof, suite green (final full suite once at the end is fine if fixes are
   harness-only).
4. Closing verdict with numbers: can code-only safe fixes bring the RAW
   workload under **57 s** (≥5× margin), or does front-end extraction remain
   deployment-mandatory? Update the eval log (dated entry) and, if the raw
   verdict changes, flag the north-star row for the arbiter — don't edit the
   verdict cell yourself.
5. Budget: ~1 day. If a single profile+sweep cycle exceeds ~2 h of machine
   time, report before continuing.
