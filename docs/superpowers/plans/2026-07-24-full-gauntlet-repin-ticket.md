# Implementer prompt — full-gauntlet re-pin at current master (release-gate artifact)

Status: ready to hand off. Paste everything below the line. Origin: every
headline row was measured across a chain of per-wave diffs (fix waves,
Cl-4 adoption, batch-1, Murty #34); no single "final master vs recorded
baselines" sweep exists, and the Murty merge (106da6f) officially staled
the ADR-0003 aggregates (13.75 / 15.49 / 9.53). This sweep is the one
artifact that closes both. Budget ~1 day. MEASUREMENT ONLY — no tuning,
no behavior code; any code you touch is bench/tooling and must be proven
byte-identical on tracker output.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-repin -b full-gauntlet-repin` off current
master, own build dir, fixtures inner-level symlinked — partially-tracked
dirs `philos/`, `sim_multisensor/`, `rbad/` need per-entry inner symlinks;
set `SIMMS_DIR`/`RBAD_DIR`; suite under `NAVTRACKER_REQUIRE_FIXTURES=1`,
0 skips. Commit on your branch; never merge/push master. Tear down your
`../navtracker-murty` + `../navtracker-murtybase` worktrees first.)

## The job

Re-run the promotion-dossier gauntlet (the 2026-07-10 precedent,
`docs/superpowers/plans/2026-07-09-pmbm-promotion-dossier-ticket.md`) at
one commit = current master. Deployable config
(`imm_cv_ct_pmbm_coverage_land_ivgate`) is the headline; the standard
comparison set (MHT canonical + the named PMBM variants) rides for the
dossier tables. Workloads: philos, harbor (5-seed, extractor KEYED BY
SEED — mean, not last-seed), env-1 + env-2 + autoferry family,
sim_multisensor (seeded), Imazu 22, HAXR arms, rbad consistency arm.
Record seeds + commit sha in every table header.

## Classification duty (the point of the exercise)

Diff EVERY row against its last recorded value (2026-07-10 dossier,
ADR-0003 + its addenda, wave-6 HAXR honest baseline, Cl-4 adoption rows,
Imazu forensics, batch-1/Murty write-ups). Each delta gets exactly one of:

1. **EXPECTED** — attributed to a named merged change, with the sha:
   M5 (philos collapse ~card_err ≈0 / gospa_false −76%; the named K=1
   cost on sc2/sc16/imazu_12/18; imazu_15/22 improved), W6.2 HAXR truth
   re-baseline, Cl-4 W25 (env-2 revival), F2 (non-deployed source-aware
   configs only), W5.5 recalibrations.
2. **FINDING** — anything else, any size: STOP-AND-REPORT to the arbiter
   before writing further conclusions. No threshold below which a delta
   is "noise" unless the row is documented knife-edge (#21) — those get
   banded language, not point claims.

Pre-registered expectations (write them in the doc BEFORE the runs):
env-2 dwelling revival stays 8/8; harbor mean ≈9.5 band; philos improves
per M5; sc2/sc16/imazu_12/18 carry the named cost; determinism replay
identical; everything else matches its recorded row.

## Deliverables

1. `docs/baselines/2026-07-24_full_gauntlet_repin.md` — the one-commit
   release-gate artifact: full tables, every delta classified, seeds/sha
   pinned, wall-clock + worst-scan latency columns for the deployable
   config (R3 envelope check).
2. Updated headline tables in `docs/algorithms/comparison-baselines.md`
   (Cl-1..Cl-4 rows that reference stale numbers get the re-pinned value
   + date). The ADR-0003 headline re-pin itself is ARBITER paper — hand
   me the numbers, don't edit the ADR.
3. Eval-log entry. Full suite green 0-skip strict as the sanity gate.
4. Commit on your branch; do not merge or push master.
