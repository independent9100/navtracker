# Implementer prompt — PMBM runtime probe: profile + knob sweep (Steps 1+2 of backlog-#21-to-be)

Status: ready to hand off. Paste everything below the line to the implementer
agent. Origin: 2026-07-05 decision — PMBM wall time now throttles development
(increment-8: EKF+GNN 33 s vs PMBM+occupancy 95 min on the same 285 s window)
and the deployment realtime margin is unproven. Step 3 (code optimization) is
explicitly NOT this ticket — it gets decided by the arbiter with this ticket's
evidence in hand.

---

You are working in the navtracker repo (C++17, CMake+Conan, read `CLAUDE.md`
first). Your task is MEASUREMENT ONLY: find out where PMBM's time actually
goes (Step 1) and what the existing config knobs buy on the compute-vs-
accuracy frontier (Step 2). No optimization code. Two hard framing rules:

- **"It's Murty" is a hypothesis, not a fact.** ~150 Bernoullis × ~100–169
  measurements is also ~15–25k Kalman/IMM updates per scan building the cost
  matrix, plus Eigen allocation churn, plus hypothesis bookkeeping. The whole
  point of Step 1 is to replace the guess with a ranking.
- **Nothing is promoted by this ticket.** Step 2 produces a frontier table and
  recommendations; config changes to defaults go through the standing gates
  in a later, separate decision.

## Setup — PARALLEL work, worktree isolation (mandatory)

The main working tree is HOT (backlog #20 is mid-implementation across many
files). Do not build or commit there.

1. `git worktree add ../navtracker-runtime-probe -b pmbm-runtime-probe`
   (branch off current master).
2. Own build dir inside the worktree. Two build-config notes:
   - Profiling needs symbols: configure with
     `-DCMAKE_BUILD_TYPE=RelWithDebInfo` and add
     `-fno-omit-frame-pointer` (so `perf` call graphs resolve without DWARF
     unwinding cost). Verify optimization level is still -O2 — profiling an
     -O0 build measures the wrong program.
   - Conan cache gotcha: `conan install` writes `~/.conan2` and fails under
     the command sandbox — run it with the sandbox disabled (see
     `docs/`/CLAUDE.md build notes), then cmake/ctest run sandboxed fine.
3. All commits go to the `pmbm-runtime-probe` branch: results docs,
   eval-log entry, and (only if needed) an additive timing/introspection
   flag. Nothing else.
4. CPU contention: the increment-8-style runs and any other heavy jobs skew
   wall-time numbers. Run the profile and the sweep on a quiet machine (or
   note contention explicitly in the results). Hotspot *ranking* tolerates
   noise; the frontier's wall-time numbers tolerate less.

## Workload (both steps use the same one)

One decimated station window from increment 8: `kattwyk_08`, the md5-pinned
decimated fixture (55–100 plots/scan regime), 285 s window, config
`imm_cv_ct_pmbm_coverage_land` (the increment-8 arm), driven through the
bench runner with the `--config-eq` / `--scenario-eq` exact filters (shipped
5ae6117). Record the baseline wall time + peak RSS for this exact run first —
it is the denominator for everything else.

## Step 1 — profile (~half day)

1. `perf record -g` (frame-pointer call graphs) on the baseline run;
   `perf report` top-down + bottom-up.
2. Classify the top hotspots into buckets:
   - cost-matrix construction (KF/IMM predicted-measurement + likelihood per
     (Bernoulli, measurement) pair),
   - assignment/Murty (k-best enumeration itself),
   - hypothesis bookkeeping (enumeration, pruning, normalization, merge),
   - occupancy/static layer,
   - memory (allocation/deallocation, Eigen temporaries — visible as
     malloc/free/memcpy in the profile),
   - other (name it).
3. Deliverable: a table (bucket → % cycles inclusive, top 3 symbols each) +
   one paragraph answering "is it Murty or the cost matrix?" with a number.
4. If phase-level timers already exist from the Cl-3 timing work, reconcile
   them against perf (two instruments agreeing = trust; disagreeing = finding).

## Step 2 — knob sweep (~1 day, runs mostly unattended)

1. Enumerate the compute-relevant knobs from `PmbmTracker::Config` at HEAD.
   Known candidates (verify names): `gate_threshold` (χ² gate), `r_min`
   (pruning floor), Murty k / global-hypothesis caps (find the actual
   fields), recycling threshold, `output_merge_max_hyp_support`,
   birth-related caps if they bound the phantom population. Pick the 4–6
   most plausible cost drivers; say which you excluded and why.
2. One-factor-at-a-time around the default: 2–3 values per knob on the
   standard workload. Record wall time, peak RSS, and the standing metric
   columns (gospa, card_err, lifetime, id_switches) for each.
3. Combine the 2–3 biggest cheap wins into one or two candidate "fast"
   configs; run those on the SAME workload plus — this is the honesty step —
   the standing gate suite (harbor_complete_truth, dense_clutter(+datum),
   and the philos label replays for KEEP safety). Accuracy deltas are
   REPORTED AND PRICED, never hidden: a knob that buys 3× at
   `tracks_on_keep` −5% is a finding, not a recommendation.
4. Deliverables:
   - `docs/baselines/2026-07-05_pmbm_runtime_frontier.csv` (or dated) — the
     frontier table, plus a short results md next to it.
   - A recommended **fast-dev config** (for A/Bs and hour-scale replays —
     does NOT need gate-clean accuracy, needs honest labels), and, IF one
     exists, a candidate default-config change (gates green) — flagged for
     the arbiter, not applied.
   - Eval-log entry with both.
5. Budget guard: bound the sweep to ~15–20 runs total; sequential overnight
   is fine. If a single run exceeds ~30 min even decimated, say so first —
   that number alone reshapes the sweep design.

## Acceptance

1. Step-1 hotspot table + the Murty-vs-cost-matrix answer, committed on the
   branch with the raw `perf` summary.
2. Step-2 frontier table + fast-dev recommendation + priced gate results for
   the candidate configs; eval-log entry.
3. Zero behavior changes: no default touched, determinism test untouched,
   full suite green in the worktree if any flag was added (skip the full
   suite if the diff is docs-only).
4. Handoff summary states: the hotspot ranking, the single best
   compute-per-accuracy knob, the fast-dev config's speedup factor, and your
   recommendation FOR OR AGAINST proceeding to Step-3 code work — with the
   profile as the argument either way.
