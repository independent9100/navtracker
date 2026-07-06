"""navtracker multi-sensor simulation fixture generator.

Seeded, deterministic offline generation of per-sensor CSV fixtures plus a
complete independent-truth CSV, consumed by the C++ ``SimMultisensorScenarioRun``
bench scenario. See README.md for the regeneration contract and the design
ticket ``docs/superpowers/plans/2026-07-06-multisensor-sim-gates-ticket.md``.

Nothing here enters the delivered navtracker targets: this is generation tooling
only (extraction-boundary ruling, 2026-07-06).
"""
