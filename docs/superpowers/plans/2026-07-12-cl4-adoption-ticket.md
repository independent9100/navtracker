# Implementer prompt — Cl-4 adoption: W_off=25 m on the deployable config + ADR-0003 + gauntlet freeze

Status: ready to hand off. Paste everything below the line. Origin: Cl-4
endgame, user decision 2026-07-12 — adopt option (a): the one deployable
PMBM config revives near-shore births by narrowing the blocked offshore
strip to 25 m at today's floor (0.10). Fully measured basis:
`docs/baselines/2026-07-11_cl4_cliff_price_list.md` (Parts 1+2, incl. the
phantom map) and `docs/baselines/2026-07-12_cl4_pending_band_probe.md`
(shape collapse + the pending-band third operating point, PARKED). This
ticket turns the measured cell into the shipped default, freezes the
gauntlet, and records the whole arc as ADR-0003. Budget ~2–3 days.
North-star tag: Cl-4 — this closes the claim.

---

You are working in the navtracker repo (C++17, read `CLAUDE.md` incl. the
second-order fixture trap; worktree `git worktree add ../navtracker-cl4a
-b cl4-adoption`, own build dir, fixtures inner-level symlinked, 0-skip
runs; commit on your branch, never merge/push master). You can remove the
merged `cl4pb` and `cl4c` worktrees first.

## Stage 1 — the change (small, scoped, per-instance)

Set `offshore_halfwidth_m = 25` on the **Cl-4 candidate config only**
(`imm_cv_ct_pmbm_coverage_land_ivgate` — the named deployable config), as
an explicit per-config constructor parameter. Do NOT change the
`CoastlinePriorParams` struct default (50/50 stays for back-compat and for
every other named config — `coverage_land`, `pmbm_land`, and the bench
comparators must be byte-identical to master; prove it). No global/static
anything (house rule). The env knobs from the sweep stay as research
levers, untouched.

## Stage 2 — tests + gauntlet freeze

1. **Zone tests (per #24-valid shapes):** companion tests to the ADR-0001
   validator — a synthetic vessel at ~30 m offshore now BIRTHS under the
   candidate config (the point of the change), a vessel/return at ~10 m is
   still suppressed, and a vessel at 60 m is untouched (the existing
   validator). Banded/structural assertions; teeth-prove the 30 m test by
   flipping W back to 50 (red) and reverting.
2. **Full promotion-gauntlet ceremony at one commit:** all seven workloads,
   three columns — champion `imm_cv_ct_mht`, prior candidate (W50), new
   candidate (W25). Expected: byte-identical to the already-measured sweep
   cells (env-2 8/8 / GOSPA 13.38; philos card_err +17.35 with the phantom
   map strip-confined; harbor 9.53 and env-1 16.57 UNCHANGED; Imazu/
   autoferry/sim rows unchanged). If any number does NOT reproduce the
   sweep cell, STOP-AND-REPORT — do not freeze a surprise.
3. **Freeze discipline:** the freeze/baseline commit waits for the FULL
   suite (frozen + unverified must not co-exist); full suite green at
   0 skips; byte-identical proof for every untouched config (the R3
   two-class A/B method).

## Stage 3 — ADR-0003 + docs (same PR; incomplete without)

1. **`docs/adr/0003-near-shore-birth-policy-25m-strip.md`** — the record
   the whole arc earned. Must contain:
   - *Context:* Cl-4 (one deployable config, user-set 2026-07-10); the
     <50 m no-birth zone (ADR-0001) collapsed the coverage stack in the
     Cl-1 env-2 channel (zero tracks).
   - *Today's prior default and why it existed:* W_off=50 + gate==target
     was deliberate (ADR-0001) — it bought the philos shore-clutter win;
     the zone was its accepted cost, priced on GOSPA alone.
   - *The measured dead-ends (each with its baseline link):* A1
     gate-lowering (philos 73.1→100); unconditional A2 floor (+40.15); A3
     sensor-typed exemption (camera = clutter source near shore, guard
     unscoreable cross-workload); the kinematic conditional floor
     (association launders a linear-structure walk into a CV transit —
     honest tiers 104–130 m); the occupancy floor-veto (race won on the
     pier but blinds a vessel transiting past structure); ramp/bar SHAPE
     (collapse proof — r_new pinned pre-suppression, shapes reduce to a
     boundary).
   - *Decision:* W_off=25 m at floor 0.10 on the deployable config. The
     evidence: vessels ride 6–42 m offshore (median 25–31 m), philos
     clutter is densest 0–10 m; the phantom map shows the W25 cost lands
     in-strip (max 264 m from shore, open-water field FLAT) while the
     floor-lowering alternative spills 5 km flyers.
   - *Accepted costs (named, dated):* (i) philos-regime harbors gain
     ~+10.45 phantom tracks/scan, in-strip — the user re-priced this
     2026-07-11/12 as acceptable (near-land waters are operator-supervised;
     an invisible real moving vessel is the worse failure per ADR-0002's
     own principle, and movers have no static-hazard fallback); this is a
     recorded deviation from the Cl-4 "~10% everywhere" definition-of-done
     on the philos row. (ii) A residual 0–25 m blind band remains: a
     vessel that stays within 25 m of shore its entire time still never
     births.
   - *The parked third operating point — the pending band:* in-band births
     admitted only after K re-detection scans; measured 2026-07-12 — beats
     unconditional-A2 at any K≥2, matches the W25 front at K≈5–8, covers
     the FULL 0–50 m band at ~7–17 s first-track latency, phantoms stay
     in-strip. PARKED, not rejected. *Reopen trigger:* a deployment that
     operates vessels inside the 0–25 m band (quay/dock operations) — then
     the pending band is the priced escalation, and its bounded latency is
     consistent with ADR-0002's own bounded-latency language.
   - *Revisit-when:* the trigger above; or a real near-shore workload
     shows the philos in-strip phantom price materially mis-estimated.
2. **ADR-0001:** add a status line — "partially superseded by ADR-0003
   (2026-07-12): the 50 m no-birth zone is narrowed to 25 m on the
   deployable config; the gate==target mechanism and the philos rationale
   stand." Do not rewrite its body.
3. **North-star (`comparison-baselines.md`):** flip the Cl-4 claim card —
   closed 2026-07-12 with the config named, the gauntlet table referenced,
   and the dated philos deviation + 0–25 m residual stated on the card
   (honest claim, not a clean sweep). Update the milestone-table row.
4. **Integration guide:** the candidate config's appendix row (W value) +
   a short consumer note in the near-shore/land section: what the 25 m
   strip means operationally (expect in-strip phantoms in cluttered
   harbors; 0–25 m shore-huggers do not initiate; pointer to ADR-0003).
5. **Learning docs:** update the land-prior/no-birth-zone passage (easy
   English) — the zone is now 25 m on the deployable config, why, and the
   pending-band idea in one paragraph (a "waiting room" for near-shore
   contacts, parked). Figure only if an existing one shows the 50 m band —
   then regenerate via `figures/generate.py`, never hand-edit.
6. **`pmbm-design.md` §10 (land prior):** math/assumptions/rationale/
   ways-to-improve updated — the four-part standard.
7. **Eval-log entry:** the freeze, with reproduce commands.

## Acceptance

1. Stage-1 diff scoped to the candidate config; every other config proven
   byte-identical.
2. Zone tests with teeth proofs; gauntlet table reproduces the sweep cells
   exactly; full suite green at 0 skips.
3. ADR-0003 + ADR-0001 note + claim-card flip + guide + learning +
   pmbm-design + eval-log all on the branch.
4. Stop-and-report: any gauntlet number deviating from the measured sweep
   cell; any untouched config not byte-identical.
5. Commit on your branch; do not merge or push master.
