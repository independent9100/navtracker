# navtracker Foundations — Learning Track

This series exists to make every engineer on the team able to read the
navtracker code with understanding, and to argue about it on a level
playing field — even if you have **never seen a Kalman filter, Bayes'
rule, or a multi-target tracker before**.

We assume you know basic probability and statistics (what a mean and
variance are, what a normal distribution is). Nothing more. From there
we build up, step by step, to **everything that is actually in this
repository**: EKF, UKF, particle filter, IMM, gating, GNN, Hungarian,
JPDA, clutter modelling, MHT, NEES/NIS, multi-sensor bias estimation,
and CPA collision-risk evaluation.

We try to write in **easy English** because many readers are not
native English speakers. Short sentences. Words explained the first
time they appear. Math is always followed by an explanation in
plain words.

## How to read this

You can read top to bottom — that is the recommended order. Each
chapter builds on the previous ones. But each chapter is also
designed to be read alone, so if you only need MHT you can jump
straight there and the prerequisites are listed at the top.

Every chapter follows the same skeleton:

1. **What problem are we solving?** — the question, in plain words.
2. **How it works** — the recipe, with pictures.
3. **Why it works** — the intuition and (light) math.
4. **What we assume** — the small print. The things that must be
   true for this to be valid.
5. **Why we can use this here** — why the assumptions hold (well
   enough) for *our* maritime fusion problem.
6. **Where this lives in the repo** — file/class pointers.
7. **What we did not pick, and why** — the alternatives we considered.

This matches the documentation standard in `CLAUDE.md`
(Math / Assumptions / Rationale / Ways to improve).

## Reading order

| #  | Chapter                                                                 | What you take away                                       |
|----|-------------------------------------------------------------------------|----------------------------------------------------------|
| 01 | [Introduction to maritime tracking](01-introduction.md)                 | Why fusing AIS/ARPA/EO-IR is hard. The big picture.      |
| 02 | [Probability refresher](02-probability-refresher.md)                    | Means, covariance, Gaussians, conditional probability.   |
| 03 | [Bayes' rule and recursive estimation](03-bayes-and-recursion.md)       | The single idea behind every filter in this codebase.    |
| 04 | [The Kalman filter](04-kalman-filter.md)                                | The classic. Predict / Update. Linear case.              |
| 05 | [The Extended Kalman Filter (EKF)](05-extended-kalman-filter.md)        | KF for nonlinear sensors. Jacobians, range/bearing.      |
| 06 | [The Unscented Kalman Filter (UKF)](06-unscented-kalman-filter.md)      | Sigma points. When EKF is not enough.                    |
| 07 | [Particle filters](07-particle-filter.md)                               | When even UKF is not enough. Sampling-based filters.     |
| 08 | [Motion models](08-motion-models.md)                                    | CV, CV5, coordinated turn, what process noise really is. |
| 09 | [Interacting Multiple Models (IMM)](09-imm.md)                          | One vessel can be straight-line *and* turning. Mixing.   |
| 10 | [Measurement models, frames, time](10-measurements-frames-time.md)      | ENU, datum, sensor pose, why we care so much about time. |
| 11 | [Gating + GNN + Hungarian](11-gating-gnn-hungarian.md)                  | Which measurement belongs to which track?                |
| 12 | [Joint Probabilistic Data Association (JPDA)](12-jpda.md)               | When you cannot decide, share the measurement softly.    |
| 13 | [Clutter and detection models](13-clutter-and-detection.md)             | False alarms, missed detections, P_D, clutter maps.      |
| 14 | [Multiple Hypothesis Tracking (MHT)](14-mht.md)                         | Defer the decision. Hypothesis trees, Murty, pruning.    |
| 15 | [Track lifecycle and scoring](15-track-lifecycle.md)                    | When is a track born, confirmed, deleted?                |
| 16 | [Filter consistency: NEES / NIS](16-nees-nis.md)                        | Is the filter lying about its own uncertainty?           |
| 17 | [Multi-sensor fusion and bias](17-multi-sensor-and-bias.md)             | Combining different sensors. Heading bias estimator.     |
| 18 | [CPA and collision risk](18-cpa-and-collision-risk.md)                  | Closest Point of Approach. The user-facing output.       |
| 19 | [Glossary](19-glossary.md)                                              | Every symbol and acronym in one place.                   |
| 20 | [Tracker performance metrics](20-tracker-metrics.md)                    | RMSE / OSPA / GOSPA / lifetime. How we score a tracker.  |
| 21 | [Inter-sensor registration bias](21-sensor-registration-bias.md)        | Per-sensor mounting offsets. The next layer after heading. |
| 22 | [Tracker stacks compared](22-tracker-stack-alternatives.md)             | Vertical slices: IMM+TOMHT vs JIPDA+VIMM vs PMBM — what's a layer, what's a fork. |
| 23 | [Poisson Multi-Bernoulli Mixture (PMBM)](23-pmbm.md)                    | RFS thinking. PPP for undetected + MBM for detected. Why slices 4-6 collapse. |
| 24 | [Coverage / Visibility Channel](24-coverage-visibility-channel.md)      | Did the sensor actually have a chance to see the target? Surveillance vs cooperative-announce. Duty-cycle misses. Comms-loss vs existence. |
| 25 | [Suppressing tracks on land: the coastline clutter prior](25-land-clutter-prior.md) | Why shore returns cause phantom tracks. The shoreline ramp c(d). Soft suppression at the waterline; inland hard gate. Why births not λ_C. |
| 26 | [Static obstacles: charted hazards as a vessel-birth prior](26-static-obstacles.md) | Two meanings of "static" (stopped vessel vs fixed environment). Charted obstacles: hard footprint core (birth-drop zone) + soft keep-clear ring (birth-weaken zone). Combined birth scale. Keep-clear proximity alarm (range check, not CPA). |
|    | [**→ Seeing the tracker in Foxglove**](11-gating-gnn-hungarian.md#9-seeing-the-tracker-in-foxglove) *(§9 of chapter 11)* | How to read covariance ellipses, gates, association lines, and NIS plots in Lichtblick. [`figures/seeing-the-tracker.png`](figures/seeing-the-tracker.png) |

## A note on diagrams

We use two kinds of diagrams in these documents:

- **Mermaid** code blocks — render automatically in GitHub, VS Code,
  and most modern markdown viewers. We use this for flowcharts,
  state machines, and sequence diagrams. If your viewer does not
  render mermaid, copy/paste into <https://mermaid.live>.
- **PNG figures** under [`figures/`](figures/) — real matplotlib
  plots (Gaussian curves, covariance ellipses, particle clouds,
  IMM mode probabilities, NIS histograms, CPA trajectories…). They
  are produced by [`figures/generate.py`](figures/generate.py); see
  [`figures/README.md`](figures/README.md) to regenerate or add new
  ones. **Do not edit the PNGs by hand** — change the script.

## A note on math

We always introduce a symbol the first time we use it. We re-introduce
it across chapters because nobody can keep 40 symbols in their head.
A full symbol reference is in chapter 19.

When a derivation is long, we show the **intuition first**, then the
math, and then the conclusion. If you trust the intuition, you can
skip the algebra.

## Cross-reference with the rest of the docs

The `docs/algorithms/` series is the **precise reference** — it has
the exact equations the code implements, with the four-section
template (Math / Assumptions / Rationale / Ways to improve). Those
documents are for someone who already knows what an EKF is and wants
to know *what choice navtracker made*. This series is for someone
who wants to know *what an EKF even is, and why we have one*.

Read this series first. Then `docs/algorithms/`. Then the code.
