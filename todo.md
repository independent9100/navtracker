# Task 0
State vectors actually used
The state vector is what the filter tracks. Common choices, in order of increasing complexity:
Constant Velocity (CV) — state is [x, y, vx, vy]. Four numbers: position and velocity in 2D. The model assumes velocity is constant and any change is "process noise." Simple, fast, perfect for cruising ships on straight courses.
Constant Acceleration (CA) — state is [x, y, vx, vy, ax, ay]. Six numbers. Now acceleration is part of the state and assumed constant, with noise driving any change. Handles a vessel that's steadily speeding up or slowing down.
Constant Turn (CT) — state is [x, y, vx, vy, ω], where ω is the turn rate (angular velocity, rad/s). Five numbers. The model assumes the target moves in a circle of constant curvature — speed stays constant in magnitude but direction rotates at rate ω. This is the workhorse for tracking turning ships and aircraft. Two sub-flavors:

CT with known ω — you fix the turn rate and run one filter per candidate ω. Crude but stable.
CT with unknown ω — ω is in the state and estimated alongside everything else. More flexible, slightly more nonlinear (which is why you'd want a UKF or EKF here, not a plain KF).

Singer model / jerk models — extend further to include rate-of-change of acceleration (jerk). Rarely needed for ships, occasionally used for highly maneuverable aircraft or missiles.
So to answer your question directly: yes, position and velocity always; acceleration when you expect speed changes; turn rate when you expect course changes; and you only go higher-order (jerk, turn-rate-rate) for genuinely agile targets, which ships aren't.
What a typical maritime IMM looks like
A common, pragmatic setup for ship tracking is three filters in parallel:

CV — handles steady cruising. Dominates ~95% of the time for merchant traffic.
CT (with ω estimated) — takes over during turns. Often instantiated twice, one biased for left turns and one for right, or with ω free.
CA or a "high-noise CV" — catches accelerations and erratic motion that the other two can't explain.

Each of these is its own EKF or UKF, with its own state vector (different length!), its own covariance matrix, its own prediction equations. The IMM mixing step has to handle the fact that the state vectors aren't the same size — it projects between them when blending. That's a bookkeeping detail but worth knowing it's there.
What's not in the state (but matters)
Some quantities you might expect to track end up as parameters of the model rather than estimated state — meaning they're fixed by the designer rather than updated each scan:

Process noise covariance (Q) — how much you trust the motion model
Measurement noise covariance (R) — how much you trust the radar
Model transition probabilities (the matrix governing how IMM switches between CV/CT/CA)

These get tuned offline from data, not estimated online. The cleverness of IMM is that even with fixed Q's per model, the combination adapts: high Q on the CT filter lets it follow turns sharply, low Q on the CV filter keeps cruising tracks smooth, and the mixing weights shift the effective behavior as the target's motion changes.
So the short answer: position and velocity are always in there, turn rate goes in when you expect turns, acceleration when you expect speed changes — and rarely both at once in the same filter, because that's what the multiple models in IMM are for in the first place.

Heterogeneous state vectors with projection between them. I am not 100% sure how ImmEstimator handles mixing when modes have different state dimensions (CV is 4-state, CT is typically 5-state, CA is 6-state). Some IMM implementations require uniform state size and pad unused dims; some implement the proper marginal-projection mixing you described. Without checking the source I can't tell you which approach navtracker's ImmEstimator takes. The mode set used in imm_cv_ct_jpda (CV + CT) does involve different dimensions, so it must handle this somehow — but the cleanliness of that handling is something to verify before claiming the IMM is "doing the right thing" by the textbook.
1:27 PM
Good catch — this is the genuinely tricky bit of IMM that most write-ups gloss over, and you're right to want to verify it rather than assume.

What the textbook says
The Blom & Bar-Shalom IMM mixing step is:

x̂_j^0 = Σ_i μ_{i|j} · x̂_i
P_j^0 = Σ_i μ_{i|j} · (P_i + (x̂_i − x̂_j^0)(x̂_i − x̂_j^0)ᵀ)

This is a moment-matching step: you compute the mean and covariance of a mixture distribution. It assumes all the x̂_i live in the same vector space. When CV is 4-dim and CT is 5-dim, that assumption breaks immediately — you literally cannot add a 4-vector to a 5-vector.

The three approaches in real implementations
1. Uniform padding (most common, mathematically dirty). Every model uses the largest state vector (e.g. 5-dim with ω), and the CV filter just sets ω = 0 with very small process noise on it. Mixing then works straightforwardly because all vectors are the same size. The cost: the CV filter is now technically estimating a "zero turn rate with low uncertainty," which biases the mixing toward CT's ω estimate during turns. Most open-source IMM implementations do this, including filterpy's IMMEstimator and most academic code I've read. It works fine in practice for CV+CT, often poorly for CV+CA where the dimensional gap is bigger and the "missing" states matter more.

2. Proper heterogeneous mixing with explicit projection. Each model defines an "unmix" function that projects from the common mixing space down to its own state, and a "remix" function that goes the other way. The mixing is done in a maximal shared space (the union of all model states), with each model's missing dimensions filled from a prior or from another model's estimate during projection. This is described in Bar-Shalom & Li (1993), "Estimation and Tracking: Principles, Techniques, and Software," and more formally in Blom & Bloem (2007), "Exact Bayesian and particle filtering of stochastic hybrid systems." Cleaner mathematically; almost never implemented in open code because it requires per-model-pair projection logic.

3. Per-model mixing, common observation space. Each model maintains its own state in its own dimension; mixing happens only in the measurement space (position, or position + velocity). The mode probabilities are still computed from per-model likelihoods, but the state mixing is done by projecting each model's state to a common observable subspace, mixing there, and then having each model re-initialize its hidden dimensions (like ω) from its own prior. This avoids the dimensional mismatch but loses information from the unobserved dimensions. Used in some published implementations; rare in libraries.

How to tell which one navtracker uses
A few quick diagnostics on the source:

Look at the state vectors. If both CV and CT models internally hold a 5-vector with ω present in CV's state (just constrained), it's approach 1 (padding). This is by far the most likely.
Look at the mixing function. If it does straight matrix arithmetic on stacked state vectors without per-model projection callbacks, it's approach 1.
Look for project_to / project_from methods or analogous transforms on the model classes. If those exist and are called during mixing, it's approach 2.
Look at how mixing is structured. If there's a separate "measurement-space mixing" path or a reduced common-state mixing, it might be approach 3.
If the answer is "approach 1 with padding," that's not wrong — it's the standard pragmatic choice and it works. But you should know it, because:

The CV model's ω estimate is essentially fictitious. Reporting it or feeding it downstream is a bug waiting to happen.
Process noise tuning for ω in the CV model matters more than people think. Set it too low and CV resists turns wrongly during mixing; too high and CV's position estimates get noisier.
During heavy maneuvering, the bias from padding shows up as slightly delayed mode switching compared to a properly heterogeneous IMM.
What I'd recommend you verify
Before claiming the IMM is "textbook correct," check three things in the navtracker source:

State vector dimensions per model. Are they actually different, or all padded to 5/6?
The mixing function signature. Does it take a list of state vectors and assume same shape, or does it dispatch through per-model adapters?
What happens to ω in the CV model after a mixing step that includes a CT branch. If ω propagates back into CV non-trivially, you have padding-style mixing and CV's ω is implicitly being tracked.


# Task 1

Your task is to combine EKF/UKF in IMM and TOMHT. Read the following suggestions, see how it fits into 
our framework and lets make a plan to include it as a seperate tracker (ideally by reusing component we 
already have).

## Architecture: three nested layers

**TOMHT (Track-Oriented Multiple Hypothesis Tracking)** is the outermost layer. It owns the combinatorial problem: which detections belong to which existing tracks, which start new tracks, which are clutter. It maintains a forest of *track trees*; each tree's branches are alternative continuations of one track. Global consistency ("each detection used at most once") is enforced only at reporting time by solving a K-best assignment problem, typically with Murty's algorithm.

**IMM (Interacting Multiple Model)** lives inside each track-tree node. A single track is not one filter — it is an IMM running several motion models in parallel (typically CV, CT, CA) and blending their estimates. The IMM produces, for that track: predicted measurement, innovation covariance, scalar measurement likelihood, and updated state — all blended across sub-models per the current mode probabilities.

**EKF/UKF** lives inside each motion model of the IMM. Each sub-model is one EKF (or UKF) with its own state vector matched to the motion assumption:

- CV: `[x, y, vx, vy]`
- CT: `[x, y, vx, vy, ω]`
- CA: `[x, y, vx, vy, ax, ay]`

The filter handles the nonlinear polar↔Cartesian transform from radar measurements (range, bearing, optionally Doppler) to Cartesian state. Use UKF for CT (turn-rate nonlinearity is meaningful); EKF is fine for CV/CA.

## The interface that ties everything together

Each layer exposes the *same* minimal interface upward:

```
predict(dt)         → advance state to current scan time
gate(z)             → does detection z fall inside the validation region?
likelihood(z)       → p(z | this hypothesis), scalar
update(z)           → incorporate detection (or process missed detection)
state(), cov()      → current best estimate
```

This is the key idea: **TOMHT doesn't know IMM exists; IMM doesn't know whether sub-filters are EKF or UKF**. Each layer just calls the same four methods on the layer below.

- An EKF/UKF implements this interface for one motion model.
- An IMM wraps N sub-filters and implements the same interface. Its `predict` runs the IMM mixing step (mix states across models via the Markov transition matrix), then calls each sub-filter's predict. Its `likelihood(z)` is the mode-weighted sum of sub-filter likelihoods. Its `update(z)` calls each sub-filter's update, recomputes mode probabilities from sub-filter likelihoods, and re-blends.
- A TOMHT track-tree leaf holds one IMM and uses the same interface to score candidate detections and grow children.

If the code is broken at the boundaries, it is almost always because someone reached through a layer — e.g., TOMHT directly accessing an EKF, or IMM mixing being skipped during predict, or per-sub-filter likelihoods being used by TOMHT instead of the IMM-blended likelihood.

## Per-scan flow

1. **Predict** — every track-tree leaf calls `imm.predict(dt)`. Internally: IMM mixing across models, then each EKF/UKF predicts.
2. **Gate** — for each leaf, gate incoming detections against the leaf's predicted measurement and innovation covariance (from the IMM, not from any individual sub-filter).
3. **Branch** — for each leaf, create child branches: one per gated detection plus one "missed detection" child. Detections that gate into no existing track also spawn new tentative track trees.
4. **Score** — child branch score = parent score + log-likelihood from IMM for the assigned detection, OR `log(1 − P_D)` for missed detection, minus clutter density / new-target penalty terms as appropriate.
5. **Update** — each child branch updates its IMM with its assigned measurement (or processes a missed detection). IMM updates sub-filters, recomputes mode probabilities, re-blends.
6. **Global hypothesis** — pick the best consistent set of leaves (one per tree, no detection used twice) via Murty's K-best assignment over the score matrix. Report tracks from the best global hypothesis.
7. **Prune** — N-scan pruning: kill branches not on the path of the best global hypothesis from N scans ago (N typically 3–5). Merge near-identical branches. Cap leaves per tree.
8. **Birth/death** — tentative tracks → confirmed after M-of-N detections. Confirmed tracks → deleted when score drops below threshold.

## What each layer uniquely contributes

- **EKF/UKF** — nonlinear state estimation under *one* motion assumption. Polar measurement handling. Gives a Gaussian posterior and a measurement likelihood for that single assumption.
- **IMM** — adaptive motion modeling. Hides "is the target maneuvering?" from the filter layer below and "which model is right?" from the tracker layer above. Provides sharper, more honest likelihoods, which directly improve TOMHT's pruning quality — without IMM, MHT compensates by inflating process noise, ballooning gates, and surviving more bogus hypotheses.
- **TOMHT** — data association under uncertainty, deferred commitment, track birth/death, multi-target consistency. Without it, IMM tracks would be corrupted by wrong associations in clutter and have no mechanism for managing multiple targets.

The three are orthogonal: EKF/UKF handles single-model state estimation; IMM handles model uncertainty; TOMHT handles association uncertainty. Each is necessary, none is sufficient alone.

## Key references

- Bar-Shalom, Li, Kirubarajan (2001), *Estimation with Applications to Tracking and Navigation*. Full mathematical reference for KF/EKF/IMM/PDA.
- Blackman & Popoli (1999), *Design and Analysis of Modern Tracking Systems*. Standard book-length TOMHT reference.
- Blackman (2004), "Multiple Hypothesis Tracking for Multiple Target Tracking," *IEEE AES Magazine* 19(1). Accessible TOMHT overview — recommended starting point.
- Blom & Bar-Shalom (1988), "The interacting multiple model algorithm for systems with Markovian switching coefficients," *IEEE TAC* 33(8). Original IMM paper.
- Mazor, Averbuch, Bar-Shalom, Dayan (1998), "Interacting multiple model methods in target tracking: a survey," *IEEE AES* 34(1). IMM model choices and tuning.
- Julier & Uhlmann (2004), "Unscented Filtering and Nonlinear Estimation," *Proc. IEEE* 92(3). UKF reference.
- Reid (1979), "An algorithm for tracking multiple targets," *IEEE TAC* 24(6). Original (hypothesis-oriented) MHT.
- Kurien (1990), "Issues in the design of practical multitarget tracking algorithms," in *Multitarget-Multisensor Tracking: Advanced Applications* (Bar-Shalom, ed.). Canonical track-oriented MHT formulation.
- Kim, Li, Ciptadi, Rehg (2015), "Multiple Hypothesis Tracking Revisited," *ICCV*. Modern TOMHT treatment.

# Task 2
We now have a bunch of implementations of common algorithms (EKF/UKF/IMM/TOMHT/...). Try to make an in depth review
if we did not mess anything up. Also check reference implementations like pythons stone-soup package.

# Task 3

Read this paper: 'https://arxiv.org/pdf/1703.04264'.

The approach in the paper is considered the 'current state of the art' in tracking and replaces TOMHT as the outest layer.
You mean also want to checkout 'https://github.com/Agarciafernandez/MTT'.

I think its a repo of the author where he provides reference implementations for PMBM (Poisson Multi-Bernoulli Mixture Filter).

