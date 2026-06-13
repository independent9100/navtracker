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
| OOSM     | Out-Of-Sequence Measurement                                     | 10            |
| OSPA     | Optimal SubPattern Assignment (bounded multi-target metric)     | 20            |
| GOSPA    | Generalised OSPA — decomposable, unbounded multi-target metric  | 20            |
| PD       | Probability of Detection                                        | 13            |
| PDA      | Probabilistic Data Association (single-target JPDA)             | 12            |
| PDAF     | PDA Filter (PDA + KF update)                                    | 12            |
| PF       | Particle Filter                                                 | 07            |
| PMBM     | Poisson Multi-Bernoulli Mixture (RFS-style filter)              | 14            |
| PSD      | Power Spectral Density                                          | 08            |
| RBPF     | Rao-Blackwellised Particle Filter                               | 07            |
| RFS      | Random Finite Set                                               | 14            |
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

## Common phrases

- **"in-gate"** — a (track, measurement) pair whose Mahalanobis
  distance is below the gate.
- **"coasting"** — track lifecycle state where predict still
  runs but no recent update has arrived.
- **"datum"** — the origin point of the ENU local tangent plane.
- **"hexagonal architecture"** — ports-and-adapters pattern;
  the core has no I/O knowledge.
- **"sensor pose"** — the ENU position (and optionally
  orientation) of the sensor at the moment of measurement.
- **"sticky modes"** — IMM transition matrix with high diagonal
  values; mode switches are rare.
- **"track tree"** — MHT representation of a target's
  hypothesis history.
- **"global hypothesis"** — MHT term: the joint assignment of
  measurements to track-tree leaves across all targets.
- **"birth gate"** — the gating step before initiating a new
  track from an unassociated measurement.
- **"score"** — log-likelihood-ratio accumulated over a track's
  history; used for confirm/delete and as the MHT objective.

---

Previous: [18 — CPA and collision risk](18-cpa-and-collision-risk.md)
Back to: [Index](00-index.md)
