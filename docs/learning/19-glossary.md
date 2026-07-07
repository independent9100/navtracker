# 19 — Glossary

> Quick lookup for symbols and acronyms used in the series.

## Symbols

| Symbol                | Meaning                                                                  | First introduced |
|-----------------------|--------------------------------------------------------------------------|------------------|
| `x`, `x_t`            | State vector at time `t`                                                 | 03               |
| `x̂`                  | Posterior mean (filter's best guess)                                     | 04               |
| `x̂⁻`                 | Prior mean (after predict, before update)                                | 04               |
| `P`                   | Posterior covariance matrix                                              | 02, 04           |
| `P⁻`                  | Prior covariance matrix                                                  | 04               |
| `F`                   | State transition matrix (linear motion model)                            | 04               |
| `f(x)`                | Nonlinear motion function (EKF/UKF/PF)                                   | 05               |
| `H`                   | Measurement matrix (linear) or Jacobian (EKF)                            | 04, 05           |
| `h(x)`                | Nonlinear measurement function                                           | 05               |
| `z`, `z_t`            | Measurement vector                                                       | 03               |
| `ẑ`                   | Predicted measurement = `h(x̂⁻)`                                         | 05               |
| `ŷ` or `ν`            | Innovation = `z − ẑ` (bearing-wrapped)                                   | 04, 16           |
| `S`                   | Innovation covariance = `H P⁻ Hᵀ + R`                                    | 04               |
| `K`                   | Kalman gain                                                              | 04               |
| `Q`                   | Process noise covariance                                                 | 04, 08           |
| `R`                   | Measurement noise covariance                                             | 04, 10           |
| `q`                   | Process noise PSD (scalar, in `m²/s³`)                                   | 08               |
| `dt`                  | Time step between updates                                                | 04, 08           |
| `μ`                   | Mean of a Gaussian                                                       | 02               |
| `σ²`                  | Variance (scalar covariance)                                             | 02               |
| `μ_k`                 | IMM mode probability for mode `k`                                        | 09               |
| `π`                   | IMM mode transition matrix                                               | 09               |
| `d²`                  | Mahalanobis distance squared                                             | 02, 11           |
| `γ`                   | Gate threshold (chi-squared quantile)                                    | 11               |
| `P_D`                 | Probability of detection                                                 | 12, 13           |
| `λ_C`                 | Clutter density (per unit measurement volume)                            | 12, 13           |
| `β_{jt}`              | JPDA marginal: probability that `z_j` is from track `t`                  | 12               |
| `β_{0t}`              | JPDA marginal: probability that track `t` was missed                     | 12               |
| `b`                   | Heading bias estimator scalar state (radians)                            | 17               |
| `N_eff`               | Effective sample size in a particle filter                               | 07               |
| `ε_NIS`, `ε_NEES`     | Normalised Innovation / Estimation Error Squared                         | 16               |

## Acronyms

| Acronym  | Expansion                                                       | Where to look |
|----------|-----------------------------------------------------------------|---------------|
| AIS      | Automatic Identification System                                 | 01, 17        |
| ARPA     | Automatic Radar Plotting Aid                                    | 01, 17        |
| CA       | Constant Acceleration (motion model)                            | 08            |
| CFAR     | Constant False Alarm Rate (sensor-side processing; out of scope)| 13            |
| COG      | Course Over Ground                                              | 17            |
| CPA      | Closest Point of Approach                                       | 18            |
| CT       | Coordinated Turn                                                | 08, 09        |
| CV       | Constant Velocity                                               | 08, 09        |
| CV5      | Constant Velocity with turn-rate placeholder (5-state)          | 08, 09        |
| CWNA     | Continuous White-Noise Acceleration                             | 08            |
| DOF      | Degrees Of Freedom                                              | 11, 16        |
| ECEF     | Earth-Centred Earth-Fixed                                       | 10            |
| EKF      | Extended Kalman Filter                                          | 05            |
| ENU      | East-North-Up local tangent plane                               | 10            |
| EWMA     | Exponentially Weighted Moving Average                           | 13            |
| EO/IR    | Electro-Optical / Infra-Red (camera)                            | 01, 17        |
| GNN      | Global Nearest Neighbour                                        | 11            |
| GPB1/2   | Generalised Pseudo-Bayesian (older multiple-model algorithms)   | 09            |
| GPS      | Global Positioning System                                       | 17            |
| HDT      | Heading True (NMEA sentence)                                    | 17            |
| IFOV     | Instantaneous Field Of View                                     | 10            |
| IMM      | Interacting Multiple Models                                     | 09            |
| IPDA     | Integrated PDA (PDA with track-existence probability)           | 12            |
| JPDA     | Joint Probabilistic Data Association                            | 12            |
| KF       | Kalman Filter                                                   | 04            |
| LLR      | Log-Likelihood Ratio (score)                                    | 14, 15        |
| MAP      | Maximum A Posteriori                                            | 14            |
| MHT      | Multiple Hypothesis Tracking                                    | 14            |
| MMSI     | Maritime Mobile Service Identity (AIS identifier)               | 17            |
| NaN      | Not a Number                                                    | 10            |
| nCV      | Noisy CV (CV with inflated process noise)                       | 08, 09        |
| NED      | North-East-Down                                                 | 10            |
| NEES     | Normalised Estimation Error Squared                             | 16            |
| NIS      | Normalised Innovation Squared                                   | 16            |
| NMEA     | National Marine Electronics Association (sensor message format) | 10, 17        |
| MCAP     | Message-Container / Archive Protocol — time-indexed binary file format for robot/sensor data | debug-viz |
| OOSM     | Out-Of-Sequence Measurement                                     | 10            |
| OSPA     | Optimal SubPattern Assignment (bounded multi-target metric)     | 20            |
| GOSPA    | Generalised OSPA — decomposable, unbounded multi-target metric  | 20            |
| PD       | Probability of Detection                                        | 13            |
| PDA      | Probabilistic Data Association (single-target JPDA)             | 12            |
| PDAF     | PDA Filter (PDA + KF update)                                    | 12            |
| PF       | Particle Filter                                                 | 07            |
| MBM      | Multi-Bernoulli Mixture (the "detected targets" half of PMBM)   | 23            |
| PMBM     | Poisson Multi-Bernoulli Mixture (RFS-style filter)              | 23            |
| PPP      | Poisson Point Process (the "undetected targets" half of PMBM)   | 23            |
| PSD      | Power Spectral Density                                          | 08            |
| RBPF     | Rao-Blackwellised Particle Filter                               | 07            |
| RFS      | Random Finite Set                                               | 23            |
| Reg.bias | Inter-sensor registration bias (per-sensor mounting offset)     | 21            |
| RTS      | Rauch-Tung-Striebel smoother                                    | 05            |
| SNR      | Signal-to-Noise Ratio                                           | 05            |
| SOG      | Speed Over Ground                                               | 17            |
| TCPA     | Time to CPA                                                     | 18            |
| TOMHT    | Track-Oriented MHT                                              | 14, memory    |
| UD       | UD-factorisation (square-root KF variant)                       | 04            |
| UKF      | Unscented Kalman Filter                                         | 06            |
| vIMM     | "vectorial" IMM (shared state dim across modes)                 | 08, 09        |
| VS-IMM   | Variable Structure IMM                                          | 09            |
| WGS84    | World Geodetic System 1984                                      | 10            |
| χ²       | Chi-squared distribution                                        | 11, 16        |

## Tools

- **MCAP / Foxglove** — `.mcap` is a compressed, time-indexed
  binary container for multi-channel sensor data. Foxglove Studio
  and its open-source fork **Lichtblick** can open `.mcap` files
  and display SceneUpdate geometry, maps, plot panels, and log
  panels on a scrubbable timeline. navtracker's
  `FoxgloveDebugRecorder` writes a `.mcap` containing the full
  fusion pipeline (tracks, detections, gates, innovations, CPA)
  for offline debugging. See `docs/debug-visualization.md`.

## Common phrases

- **"in-gate"** — a (track, measurement) pair whose Mahalanobis
  distance is below the gate.
- **"coasting"** — track lifecycle state where predict still
  runs but no recent update has arrived.
- **"datum"** — the origin point of the ENU local tangent plane.
- **"hexagonal architecture"** — ports-and-adapters pattern;
  the core has no I/O knowledge.
- **"clutter prior"** — a spatial prior `c ∈ [0,1]` that estimates
  how likely a position is to produce false sensor returns (shore /
  structure clutter) rather than real vessel detections. Chapter 25.
- **"compound-K"** — a clutter model where the false-alarm count is a
  Poisson whose *rate is itself random* (gamma-distributed "texture" ×
  Poisson "speckle"). The marginal count is **negative-binomial** —
  heavier-tailed and clumpier than flat Poisson, so high-count scans are
  normal, not surprising. It is how the sim battery *generates* realistic
  clumpy radar clutter (`sim_ms_clutter_burst`). Chapter 13 §9.
- **"over-dispersion"** — a count distribution whose variance exceeds its
  mean (ratio > 1), i.e. clumpier than Poisson. Compound-K is over-dispersed;
  flat Poisson has variance = mean. A tracker whose clutter term assumes flat
  Poisson over-counts on over-dispersed clutter. Chapter 13 §9.
- **"shoreline ramp"** — the smooth function `c(d) = clamp((W_off−d)/(W_off+W_in), 0, 1)`
  of signed distance `d` to the nearest shore edge. Rises from 0 (open water)
  to 1 (well inland) across a configurable margin band. Chapter 25.
- **"land mask"** — colloquial name for the coastline clutter prior when viewed
  as a binary yes/no surface; in navtracker the mask is always a continuous ramp,
  not a hard binary. The inland plateau (c ≈ 1) is the "masked" region. Chapter 25.
- **"static obstacle"** — a discrete charted hazard in navigable water: a rock,
  wreck, pile, platform, or AtoN. Distinguished from the coastline (a large region)
  and from a stopped vessel (which is still tracked). Stored as `StaticObstacle`
  (WGS84 position + radii + attributes). Chapter 26.
- **"footprint (obstacle)"** — the physical extent of a charted obstacle, given
  as a radius in metres (`footprint_radius_m`). The hard no-birth core is
  `R_hard = footprint_radius_m + position_uncertainty_m`. Births inside `R_hard`
  are hard-gated (dropped). Chapter 26.
- **"keep-clear radius"** — the operator-defined safety margin around a charted
  obstacle (`keep_clear_radius_m`). Births between `R_hard` and `R_soft =
  max(keep_clear_radius_m, R_hard)` are soft-suppressed (weakened but not
  dropped). The `StaticHazardEvaluator` fires a proximity alarm when own-ship
  enters this radius. Chapter 26.
- **"AtoN"** — Aid to Navigation. A buoy, beacon, or light tower placed to guide
  vessels. AtoN types in navtracker: Real (physical object), Synthetic (virtual
  radar/AIS overlay for a physical object), Virtual (an AIS-broadcast hazard
  mark with no physical structure at that position). Encoded in `AtoNRealism`.
  AIS Message 21 broadcasts AtoN positions as a first-class `StaticObstacle`
  source. Chapter 26.
- **"bearing wedge"** — a direction-only hazard for a camera-only contact that
  cannot initiate a track (a `Bearing2D` stream has no range). A sector from
  own-ship (apex) spanning ±half-width (`= 2σ`, σ composed of camera ⊕ heading,
  with a floor) about the detection bearing, range usually unbounded. Satisfies
  ADR 0002 "never invisible" without a position; CPA not computable. A confirmed
  track on the bearing *suppresses* it (recomputed each drain, never deleted);
  only camera silence removes it. Stored as `BearingWedge` / `BearingWedgeModel`.
  Chapter 28.
- **"sensor pose"** — the ENU position (and optionally
  orientation) of the sensor at the moment of measurement.
- **"sticky modes"** — IMM transition matrix with high diagonal
  values; mode switches are rare.
- **"track tree"** — MHT representation of a target's
  hypothesis history.
- **"global hypothesis"** — MHT term: the joint assignment of
  measurements to track-tree leaves across all targets.
- **"Bernoulli component"** — in PMBM, one possible *detected* target:
  a pair `(r, f(x))` of an existence probability `r ∈ [0,1]` and a
  state density `f(x)`. A set of these is a Multi-Bernoulli; a weighted
  mixture of such sets is the MBM. Contrasted with the PPP, which holds
  the *undetected* targets. Chapter 23.
- **"birth gate"** — the gating step before initiating a new
  track from an unassociated measurement.
- **"score"** — log-likelihood-ratio accumulated over a track's
  history; used for confirm/delete and as the MHT objective.
- **"duty cycle"** — the period of one complete sensor sweep or
  rotation (e.g. 2.5 s for a radar that rotates 24 times per
  minute). One duty cycle = one opportunity to see any target
  inside coverage. See §24.
- **"cooperative-announce source"** — a sensor kind where the
  target sends its own reports on its own schedule (AIS, fleet
  link). Silence is weak evidence; reports are strong. Contrasted
  with surveillance sources. See §24.
- **"surveillance sensor"** — a sensor kind that actively searches
  an area on a known duty cycle (radar, EO/IR, lidar). Silence
  over covered ground is strong, symmetric evidence. See §24.
- **"coverage / visibility channel"** — the ISensorActivity port
  that tells the tracker which sensor had a real chance to observe
  a track during a given time window. Enables per-duty-cycle miss
  charging instead of per-blip. See §24.
- **"comms-loss signal"** — a flag raised when a cooperative
  source's own-identity report is overdue. Tells the operator
  contact was lost; does NOT lower the track's existence
  probability. See §24.
- **"occupancy grid"** — a metric grid of cells laid over the water,
  anchored to the datum; each cell holds one persistence number that
  estimates how likely fixed structure sits there. Built live from
  the PMBM clutter feed. Chapter 27.
- **"EWMA persistence"** — the running score in each occupancy-grid
  cell, updated as `p ← (1−α)·p + α·w` (an Exponentially Weighted
  Moving Average of the per-scan clutter evidence `w`). Old evidence
  fades; a cell that keeps producing clutter climbs toward 1. Chapter 27.
- **"coverage sector"** — the angular/range wedge a scanning sensor
  actually swept this scan. The occupancy model self-estimates it
  (from scanning-source returns only) so it decays persistence *only*
  over ground it truly observed, never over unswept gaps. Chapters 24, 27.
- **"conservation-by-construction"** — the ADR-0002 safety invariant:
  birth suppression at a location is legal *only* if that same
  location is simultaneously emitted as a static hazard. An object is
  never suppressed into nothing. Chapters 26, 27.
- **"non-scanning source"** — a sensor kind that reports vessel
  *positions* without sweeping a footprint (AIS, Cooperative/fleet
  GNSS). Excluded from coverage-sector estimation (a wedge fit to
  position reports over-claims coverage) and the strongest evidence
  for the suppression veto. Chapter 27. Contrast surveillance sensor.

---

Previous: [18 — CPA and collision risk](18-cpa-and-collision-risk.md)
Back to: [Index](00-index.md)
