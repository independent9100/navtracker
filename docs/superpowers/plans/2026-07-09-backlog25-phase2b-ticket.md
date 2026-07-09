# Implementer prompt — backlog #25 Phase 2b: true-innovation re-probe, then build the position-innovation gate

Status: ready to hand off. Paste everything below the line. Origin: #25
Phase 2a (`docs/baselines/2026-07-09_b25_phase2a_probe.md`, merged) — the
velocity bound is a measured NO-BUILD (false-fires on healthy tracks
carrying transient velocity spikes); the position-displacement gate PASSED
the binding criteria (5/6 dying tracks flagged before permanent gate-exit,
0.13%/0.00% false-fire) — but on a POSTERIOR-JUMP PROXY, because the diag
export lacks the true measurement innovation. This ticket closes that gap
first, then builds. North-star tag: Cl-3 (#25 = the deployment-choice
discriminator). Budget: Stage 1 ~half day, Stage 2 ~1–1.5 days, CHECKPOINT
between them.

Explicitly OUT of scope: the coalescence guard and the ambiguity-gated PDA
soft update (ranked alternates — separate tickets if ever); any change to
the miss-P_D existence recursion or the birth channel (the philos brake and
the λ_C invariant stay untouched — that is a design constraint, not advice).

---

You are working in the navtracker repo (C++17, read `CLAUDE.md`; worktree
`git worktree add ../navtracker-b25p2b -b backlog25-phase2b`, own build dir;
fixtures from the MAIN tree; skip list named in every handoff — after the
sunset episode, skips are diffed by name, and note the ctest-from-build-dir
trap: wire build/tests + build/data symlinks or run the binary from the
worktree root so philos-gated cases RUN).

## Stage 1 — export the true innovation, re-probe (still zero behavior change)

1. Extend the Phase-1 diagnostic surface (`IPmbmDiagnosticSink` /
   `PmbmScanDiag`) with, per Bernoulli per scan: the applied measurement's
   position innovation (measurement minus predicted position, and its norm),
   and the per-mode IMM weights (the Phase-2a declared gap). Additive,
   default-off, byte-identical — same standard as Phase 1 (prove it the
   same way: states.csv + metric rows identical diag on/off, one non-imazu
   config included).
2. Re-run the Phase-2a D-axis probe on the TRUE innovation
   (`tools/pmbm_phase2a_probe.py` extended or a sibling tool): does
   D_max ∈ {100, 200, 400} m on real innovation reproduce the proxy's
   verdict (≥5/6 dying tracks flagged BEFORE permanent gate-exit, <1%
   false-fire on healthy confirmed track-scans, autoferry included)?
   This is BINDING: if true-innovation flagging comes too late or
   false-fires, STOP at the checkpoint and report — do not proceed to
   Stage 2 on the proxy's strength.
3. The innovation data also answers the placement question the proxy
   cannot: does the runaway START with one oversized accepted innovation
   (→ the gate belongs at update-acceptance) or with a sequence of
   moderate ones (→ clamping/deweighting inside the estimator)? Answer it
   with the traces from imazu_15/22.

**CHECKPOINT: report Stage-1 numbers + placement verdict to the arbiter
before writing any behavior code.**

## Stage 2 — build the gate (only on arbiter go)

Design constraints, fixed in advance:

- **Clamp kinematics, never existence.** A flagged update must never
  delete or decay the Bernoulli — presence is the requirement (ADR 0002);
  the failure we are fixing is the ESTIMATE leaving, not the track dying.
  Candidate actions per the Stage-1 placement verdict: reject the update
  (track coasts, one-scan), or accept position with deweighted velocity —
  pick ONE, justify from the traces.
- **Per-instance config** (`...Config`, ctor-threaded, no global state),
  opt-in, default OFF and byte-identical when off. D_max starting band
  200–400 m from the probe; make it a config field, not a constant.
- **Gates (agreed now):** primary = imazu family, loss-seconds-overlapping-
  CPA and re-acquire-id count on the 6 dying cases, plus the census
  clutter-phantom count (the 82.5% phantom majority means the gate should
  also SHORTEN phantom lifetimes — measure it, it's the free bonus).
  No-regression wall = philos KEEP configs, HAXR, autoferry, harbor
  yardstick: GOSPA/card_err/lifetime within noise, byte-identical where
  the gate is off. Banded assertions only (#24 — no exact pins; and after
  case (5), no cross-config comparisons on marginal regions).
- **Docs (same PR):** algorithm doc with the four required sections;
  `docs/learning/` update (gating chapter — plain-English: why a huge
  position jump means "wrong measurement", with a small figure via
  `docs/learning/figures/generate.py`); integration-guide entry + config
  appendix row (the drift-guard test will remind you).

## Acceptance

1. Stage 1: extended diag export with byte-identical proof; true-innovation
   probe tables committed (`docs/baselines/2026-07-09_b25_phase2b.md` +
   dated eval-log entry); checkpoint honored.
2. Stage 2 (on go): gate implemented per constraints; A/B tables on the
   agreed gates; docs trio updated; full suite green with skips named.
3. Stop-and-report: Stage-1 binding criteria fail on true innovation; or
   the no-regression wall breaks at every D_max in the band; or the fix
   cannot avoid touching existence/birth to work.
