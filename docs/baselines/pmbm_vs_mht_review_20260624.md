# PMBM vs MHT — reviewable metric run (2026-06-24)
**Source**: filtered from `cl3_timing_pmbm_vs_mht_20260624T062343Z.csv` (pinned 23×29 sweep, seeds [0,1,2], elapsed 4028 s, 1127 cells).  
**Subset CSV**: `docs/baselines/pmbm_vs_mht_review_20260624.csv` (3 configs × 29 scenarios × 3 seeds).
**Chains**:
- `imm_cv_ct_mht` — UKF / IMM(CV+CT) / TOMHT / IPDA+VIMM, with `SensorBiasEstimator`. **Current canonical.**
- `imm_cv_ct_pmbm_adapt` — same IMM, PMBM associator, adaptive birth (Reuter 2014), λ_birth=1e-5, K=1.
- `imm_cv_ct_pmbm_adapt_k3` — same plus adaptive K=3 best with cross-parent birth-id cache.

All values are seed-averages across seeds 0,1,2. Δ% columns are (PMBM − MHT) / MHT × 100; **negative = PMBM better** for OSPA / GOSPA / id_switches / pos_rmse / sog_rmse / wall_seconds. For nees_mean, *closer to 1* is better — no sign convention.

## Per-scenario headline (gospa / id_switches / pos_rmse / wall)
| scenario | MHT gospa | PMBM-K1 gospa Δ | PMBM-K3 gospa Δ | MHT id_sw | K1 id_sw Δ | K3 id_sw Δ | MHT pos_rmse | K1 pos_rmse Δ | K3 pos_rmse Δ | MHT wall_s | K1 wall_s Δ | K3 wall_s Δ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| philos | 69.427 | +19.0% | +18.8% | 0.0 | — | — | 25.583 | -49.6% | -57.8% | 33.446 | -29.4% | +967.8% |
| autoferry_scenario13_anchored | 3.122 | +9.4% | +10.9% | 13.0 | -88.5% | -96.2% | 2.271 | -36.8% | -33.5% | 5.684 | -91.9% | -19.3% |
| autoferry_scenario16_anchored | 2.352 | +28.7% | +38.4% | 2.0 | -50.0% | -50.0% | 1.139 | +33.7% | +102.7% | 6.770 | -93.0% | -32.1% |
| autoferry_scenario17_anchored | 2.627 | +329.0% | +367.5% | 3.0 | -100.0% | -100.0% | 1.277 | +2.1% | +36.8% | 2.226 | -87.3% | +14.6% |
| autoferry_scenario22_anchored | 3.425 | +114.5% | +119.1% | 7.0 | -64.3% | -71.4% | 1.184 | +166.5% | +161.4% | 2.669 | -88.0% | +14.3% |
| autoferry_scenario2_anchored | 2.336 | +5.1% | +25.0% | 1.0 | -100.0% | -100.0% | 1.637 | +25.1% | +43.9% | 7.753 | -94.0% | -34.0% |
| autoferry_scenario3_anchored | 1.538 | +1.6% | +1.9% | 0.0 | 0.0% | 0.0% | 1.215 | +1.1% | +2.1% | 14.895 | -96.3% | -55.3% |
| autoferry_scenario4_anchored | 2.641 | -31.5% | -28.5% | 2.0 | -100.0% | -100.0% | 1.741 | -30.4% | -27.5% | 7.968 | -94.9% | -35.8% |
| autoferry_scenario5_anchored | 3.061 | -45.2% | -41.7% | 5.0 | -100.0% | -100.0% | 1.919 | -24.4% | -20.5% | 30.797 | -97.4% | -69.5% |
| autoferry_scenario6_anchored | 5.598 | -52.6% | -42.6% | 6.0 | -100.0% | -100.0% | 1.898 | +24.8% | +52.6% | 10.954 | -95.7% | -51.8% |
| autoferry_scenario13 | 21.488 | -30.0% | -31.6% | 14.5 | -75.9% | -69.0% | 8.134 | +30.9% | +24.9% | 2.821 | -87.3% | +31.2% |
| autoferry_scenario16 | 25.792 | -50.4% | -47.3% | 17.0 | -50.0% | -26.5% | 12.450 | -17.1% | +11.5% | 3.418 | -89.7% | +10.1% |
| autoferry_scenario17 | 25.202 | -36.1% | -36.2% | 31.0 | -96.8% | -80.6% | 27.804 | -39.9% | -28.0% | 0.976 | -81.4% | +96.1% |
| autoferry_scenario2 | 33.281 | -48.1% | -48.0% | 38.0 | -86.8% | -97.4% | 10.651 | -15.7% | -5.2% | 3.337 | -89.6% | +11.9% |
| autoferry_scenario22 | 36.867 | -42.0% | -42.7% | 36.0 | -48.6% | -50.0% | 27.788 | -5.4% | -6.6% | 1.909 | -87.2% | +35.1% |
| autoferry_scenario3 | 35.936 | -43.6% | -45.4% | 41.0 | -96.3% | -97.6% | 28.063 | -44.9% | -41.9% | 4.733 | -91.4% | -20.6% |
| autoferry_scenario4 | 31.939 | -59.1% | -61.4% | 16.0 | -65.6% | -81.2% | 13.785 | -30.8% | -28.0% | 2.142 | -89.9% | +34.1% |
| autoferry_scenario5 | 33.486 | -40.3% | -41.3% | 49.5 | -78.8% | -92.9% | 21.051 | -4.2% | -19.8% | 8.821 | -94.8% | -38.9% |
| autoferry_scenario6 | 30.548 | -41.6% | -43.0% | 36.0 | -59.7% | -63.9% | 28.132 | -52.2% | -53.2% | 3.211 | -87.0% | +46.5% |
| dense_clutter | 12.324 | +3.0% | +1.3% | 0.8 | -100.0% | -80.0% | 7.419 | +6.1% | +13.6% | 0.021 | -55.2% | +263.0% |
| non_cooperative | 17.900 | 0.0% | +0.1% | 0.0 | 0.0% | 0.0% | 16.126 | 0.0% | +3.7% | 0.001 | -26.4% | +488.6% |
| ais_dropout | 14.995 | +0.6% | -0.7% | 0.3 | -50.0% | 0.0% | 22.195 | -0.7% | -1.0% | 0.003 | -36.7% | +85.3% |
| clock_skew | 4.607 | -4.7% | -4.7% | 0.0 | 0.0% | 0.0% | 5.028 | +0.5% | +0.5% | 0.002 | +6.1% | +102.7% |
| crossing | 9.719 | -1.9% | -1.8% | 0.0 | 0.0% | 0.0% | 7.183 | +1.0% | +1.0% | 0.004 | -29.1% | +102.0% |
| crossing_dropout | 12.014 | -1.5% | -1.5% | 2.0 | 0.0% | 0.0% | 20.860 | -1.0% | -1.0% | 0.003 | -20.1% | +74.3% |
| head_on | 9.719 | -1.9% | -2.0% | 0.0 | 0.0% | 0.0% | 7.183 | +1.0% | +0.8% | 0.004 | -4.4% | +139.6% |
| overtaking | 6.145 | -3.3% | -3.3% | 0.0 | 0.0% | 0.0% | 4.455 | +0.7% | +0.7% | 0.007 | -16.1% | +30.1% |
| parallel_targets | 6.726 | -6.0% | -6.0% | 0.0 | 0.0% | 0.0% | 4.724 | +0.9% | +0.9% | 0.002 | -13.3% | +16.3% |
| speed_change | 5.358 | -4.0% | -4.0% | 0.0 | 0.0% | 0.0% | 5.835 | +0.0% | +0.0% | 0.002 | -28.1% | +85.7% |

## Per-scenario absolute values (all headline metrics)

### wall_seconds
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 33.446 | 23.610 | -29.4% | 357.132 | +967.8% |
| autoferry_scenario13_anchored | 5.684 | 0.462 | -91.9% | 4.590 | -19.3% |
| autoferry_scenario16_anchored | 6.770 | 0.471 | -93.0% | 4.596 | -32.1% |
| autoferry_scenario17_anchored | 2.226 | 0.282 | -87.3% | 2.551 | +14.6% |
| autoferry_scenario22_anchored | 2.669 | 0.320 | -88.0% | 3.049 | +14.3% |
| autoferry_scenario2_anchored | 7.753 | 0.468 | -94.0% | 5.120 | -34.0% |
| autoferry_scenario3_anchored | 14.895 | 0.548 | -96.3% | 6.661 | -55.3% |
| autoferry_scenario4_anchored | 7.968 | 0.404 | -94.9% | 5.117 | -35.8% |
| autoferry_scenario5_anchored | 30.797 | 0.787 | -97.4% | 9.401 | -69.5% |
| autoferry_scenario6_anchored | 10.954 | 0.469 | -95.7% | 5.274 | -51.8% |
| autoferry_scenario13 | 2.821 | 0.359 | -87.3% | 3.701 | +31.2% |
| autoferry_scenario16 | 3.418 | 0.352 | -89.7% | 3.762 | +10.1% |
| autoferry_scenario17 | 0.976 | 0.181 | -81.4% | 1.915 | +96.1% |
| autoferry_scenario2 | 3.337 | 0.347 | -89.6% | 3.734 | +11.9% |
| autoferry_scenario22 | 1.909 | 0.245 | -87.2% | 2.579 | +35.1% |
| autoferry_scenario3 | 4.733 | 0.406 | -91.4% | 3.760 | -20.6% |
| autoferry_scenario4 | 2.142 | 0.216 | -89.9% | 2.872 | +34.1% |
| autoferry_scenario5 | 8.821 | 0.461 | -94.8% | 5.387 | -38.9% |
| autoferry_scenario6 | 3.211 | 0.419 | -87.0% | 4.705 | +46.5% |
| dense_clutter | 0.021 | 0.009 | -55.2% | 0.077 | +263.0% |
| non_cooperative | 0.001 | 0.001 | -26.4% | 0.007 | +488.6% |
| ais_dropout | 0.003 | 0.002 | -36.7% | 0.006 | +85.3% |
| clock_skew | 0.002 | 0.002 | +6.1% | 0.003 | +102.7% |
| crossing | 0.004 | 0.003 | -29.1% | 0.008 | +102.0% |
| crossing_dropout | 0.003 | 0.003 | -20.1% | 0.006 | +74.3% |
| head_on | 0.004 | 0.003 | -4.4% | 0.009 | +139.6% |
| overtaking | 0.007 | 0.006 | -16.1% | 0.009 | +30.1% |
| parallel_targets | 0.002 | 0.002 | -13.3% | 0.003 | +16.3% |
| speed_change | 0.002 | 0.001 | -28.1% | 0.003 | +85.7% |

### gospa_mean
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 69.427 | 82.627 | +19.0% | 82.454 | +18.8% |
| autoferry_scenario13_anchored | 3.122 | 3.417 | +9.4% | 3.462 | +10.9% |
| autoferry_scenario16_anchored | 2.352 | 3.028 | +28.7% | 3.255 | +38.4% |
| autoferry_scenario17_anchored | 2.627 | 11.272 | +329.0% | 12.284 | +367.5% |
| autoferry_scenario22_anchored | 3.425 | 7.347 | +114.5% | 7.505 | +119.1% |
| autoferry_scenario2_anchored | 2.336 | 2.454 | +5.1% | 2.920 | +25.0% |
| autoferry_scenario3_anchored | 1.538 | 1.562 | +1.6% | 1.567 | +1.9% |
| autoferry_scenario4_anchored | 2.641 | 1.809 | -31.5% | 1.888 | -28.5% |
| autoferry_scenario5_anchored | 3.061 | 1.676 | -45.2% | 1.785 | -41.7% |
| autoferry_scenario6_anchored | 5.598 | 2.652 | -52.6% | 3.213 | -42.6% |
| autoferry_scenario13 | 21.488 | 15.046 | -30.0% | 14.689 | -31.6% |
| autoferry_scenario16 | 25.792 | 12.785 | -50.4% | 13.592 | -47.3% |
| autoferry_scenario17 | 25.202 | 16.105 | -36.1% | 16.073 | -36.2% |
| autoferry_scenario2 | 33.281 | 17.279 | -48.1% | 17.302 | -48.0% |
| autoferry_scenario22 | 36.867 | 21.386 | -42.0% | 21.132 | -42.7% |
| autoferry_scenario3 | 35.936 | 20.281 | -43.6% | 19.619 | -45.4% |
| autoferry_scenario4 | 31.939 | 13.079 | -59.1% | 12.322 | -61.4% |
| autoferry_scenario5 | 33.486 | 19.976 | -40.3% | 19.658 | -41.3% |
| autoferry_scenario6 | 30.548 | 17.846 | -41.6% | 17.399 | -43.0% |
| dense_clutter | 12.324 | 12.700 | +3.0% | 12.479 | +1.3% |
| non_cooperative | 17.900 | 17.900 | 0.0% | 17.921 | +0.1% |
| ais_dropout | 14.995 | 15.089 | +0.6% | 14.896 | -0.7% |
| clock_skew | 4.607 | 4.392 | -4.7% | 4.392 | -4.7% |
| crossing | 9.719 | 9.538 | -1.9% | 9.539 | -1.8% |
| crossing_dropout | 12.014 | 11.833 | -1.5% | 11.833 | -1.5% |
| head_on | 9.719 | 9.538 | -1.9% | 9.521 | -2.0% |
| overtaking | 6.145 | 5.945 | -3.3% | 5.945 | -3.3% |
| parallel_targets | 6.726 | 6.325 | -6.0% | 6.325 | -6.0% |
| speed_change | 5.358 | 5.143 | -4.0% | 5.143 | -4.0% |

### ospa_mean
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 424.611 | 454.804 | +7.1% | 454.611 | +7.1% |
| autoferry_scenario13_anchored | 23.920 | 46.716 | +95.3% | 45.727 | +91.2% |
| autoferry_scenario16_anchored | 19.282 | 36.498 | +89.3% | 30.379 | +57.6% |
| autoferry_scenario17_anchored | 23.643 | 271.083 | +1046.6% | 295.856 | +1151.3% |
| autoferry_scenario22_anchored | 46.711 | 151.031 | +223.3% | 158.871 | +240.1% |
| autoferry_scenario2_anchored | 9.295 | 12.572 | +35.3% | 13.230 | +42.3% |
| autoferry_scenario3_anchored | 1.364 | 3.039 | +122.8% | 3.213 | +135.6% |
| autoferry_scenario4_anchored | 20.117 | 7.008 | -65.2% | 7.488 | -62.8% |
| autoferry_scenario5_anchored | 18.570 | 4.309 | -76.8% | 4.836 | -74.0% |
| autoferry_scenario6_anchored | 86.137 | 13.945 | -83.8% | 15.890 | -81.6% |
| autoferry_scenario13 | 310.787 | 199.485 | -35.8% | 198.729 | -36.1% |
| autoferry_scenario16 | 355.168 | 234.858 | -33.9% | 207.554 | -41.6% |
| autoferry_scenario17 | 303.050 | 310.265 | +2.4% | 301.064 | -0.7% |
| autoferry_scenario2 | 381.990 | 230.957 | -39.5% | 213.829 | -44.0% |
| autoferry_scenario22 | 415.894 | 293.347 | -29.5% | 289.314 | -30.4% |
| autoferry_scenario3 | 398.638 | 272.633 | -31.6% | 271.402 | -31.9% |
| autoferry_scenario4 | 388.034 | 156.023 | -59.8% | 134.437 | -65.4% |
| autoferry_scenario5 | 384.268 | 243.602 | -36.6% | 260.696 | -32.2% |
| autoferry_scenario6 | 342.257 | 298.092 | -12.9% | 297.536 | -13.1% |
| dense_clutter | 102.322 | 108.642 | +6.2% | 113.078 | +10.5% |
| non_cooperative | 359.049 | 359.049 | 0.0% | 361.529 | +0.7% |
| ais_dropout | 51.780 | 42.525 | -17.9% | 39.569 | -23.6% |
| clock_skew | 16.753 | 4.392 | -73.8% | 4.392 | -73.8% |
| crossing | 19.039 | 6.765 | -64.5% | 6.765 | -64.5% |
| crossing_dropout | 24.417 | 12.142 | -50.3% | 12.142 | -50.3% |
| head_on | 19.039 | 6.765 | -64.5% | 6.753 | -64.5% |
| overtaking | 12.443 | 4.204 | -66.2% | 4.204 | -66.2% |
| parallel_targets | 20.951 | 4.472 | -78.7% | 4.472 | -78.7% |
| speed_change | 17.505 | 5.143 | -70.6% | 5.143 | -70.6% |

### id_switches
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 0.0 | 0.1 | — | 0.1 | — |
| autoferry_scenario13_anchored | 13.0 | 1.5 | -88.5% | 0.5 | -96.2% |
| autoferry_scenario16_anchored | 2.0 | 1.0 | -50.0% | 1.0 | -50.0% |
| autoferry_scenario17_anchored | 3.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry_scenario22_anchored | 7.0 | 2.5 | -64.3% | 2.0 | -71.4% |
| autoferry_scenario2_anchored | 1.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry_scenario3_anchored | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| autoferry_scenario4_anchored | 2.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry_scenario5_anchored | 5.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry_scenario6_anchored | 6.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry_scenario13 | 14.5 | 3.5 | -75.9% | 4.5 | -69.0% |
| autoferry_scenario16 | 17.0 | 8.5 | -50.0% | 12.5 | -26.5% |
| autoferry_scenario17 | 31.0 | 1.0 | -96.8% | 6.0 | -80.6% |
| autoferry_scenario2 | 38.0 | 5.0 | -86.8% | 1.0 | -97.4% |
| autoferry_scenario22 | 36.0 | 18.5 | -48.6% | 18.0 | -50.0% |
| autoferry_scenario3 | 41.0 | 1.5 | -96.3% | 1.0 | -97.6% |
| autoferry_scenario4 | 16.0 | 5.5 | -65.6% | 3.0 | -81.2% |
| autoferry_scenario5 | 49.5 | 10.5 | -78.8% | 3.5 | -92.9% |
| autoferry_scenario6 | 36.0 | 14.5 | -59.7% | 13.0 | -63.9% |
| dense_clutter | 0.8 | 0.0 | -100.0% | 0.2 | -80.0% |
| non_cooperative | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| ais_dropout | 0.3 | 0.2 | -50.0% | 0.3 | 0.0% |
| clock_skew | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| crossing | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| crossing_dropout | 2.0 | 2.0 | 0.0% | 2.0 | 0.0% |
| head_on | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| overtaking | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| parallel_targets | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |
| speed_change | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |

### pos_rmse_m
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 25.583 | 12.901 | -49.6% | 10.796 | -57.8% |
| autoferry_scenario13_anchored | 2.271 | 1.435 | -36.8% | 1.510 | -33.5% |
| autoferry_scenario16_anchored | 1.139 | 1.523 | +33.7% | 2.310 | +102.7% |
| autoferry_scenario17_anchored | 1.277 | 1.304 | +2.1% | 1.747 | +36.8% |
| autoferry_scenario22_anchored | 1.184 | 3.156 | +166.5% | 3.096 | +161.4% |
| autoferry_scenario2_anchored | 1.637 | 2.049 | +25.1% | 2.356 | +43.9% |
| autoferry_scenario3_anchored | 1.215 | 1.228 | +1.1% | 1.241 | +2.1% |
| autoferry_scenario4_anchored | 1.741 | 1.213 | -30.4% | 1.263 | -27.5% |
| autoferry_scenario5_anchored | 1.919 | 1.450 | -24.4% | 1.526 | -20.5% |
| autoferry_scenario6_anchored | 1.898 | 2.368 | +24.8% | 2.895 | +52.6% |
| autoferry_scenario13 | 8.134 | 10.649 | +30.9% | 10.157 | +24.9% |
| autoferry_scenario16 | 12.450 | 10.324 | -17.1% | 13.888 | +11.5% |
| autoferry_scenario17 | 27.804 | 16.715 | -39.9% | 20.030 | -28.0% |
| autoferry_scenario2 | 10.651 | 8.975 | -15.7% | 10.096 | -5.2% |
| autoferry_scenario22 | 27.788 | 26.281 | -5.4% | 25.952 | -6.6% |
| autoferry_scenario3 | 28.063 | 15.458 | -44.9% | 16.315 | -41.9% |
| autoferry_scenario4 | 13.785 | 9.533 | -30.8% | 9.926 | -28.0% |
| autoferry_scenario5 | 21.051 | 20.164 | -4.2% | 16.875 | -19.8% |
| autoferry_scenario6 | 28.132 | 13.440 | -52.2% | 13.159 | -53.2% |
| dense_clutter | 7.419 | 7.874 | +6.1% | 8.432 | +13.6% |
| non_cooperative | 16.126 | 16.126 | 0.0% | 16.722 | +3.7% |
| ais_dropout | 22.195 | 22.049 | -0.7% | 21.984 | -1.0% |
| clock_skew | 5.028 | 5.053 | +0.5% | 5.052 | +0.5% |
| crossing | 7.183 | 7.253 | +1.0% | 7.255 | +1.0% |
| crossing_dropout | 20.860 | 20.654 | -1.0% | 20.654 | -1.0% |
| head_on | 7.183 | 7.253 | +1.0% | 7.242 | +0.8% |
| overtaking | 4.455 | 4.485 | +0.7% | 4.485 | +0.7% |
| parallel_targets | 4.724 | 4.768 | +0.9% | 4.768 | +0.9% |
| speed_change | 5.835 | 5.838 | +0.0% | 5.837 | +0.0% |

### sog_rmse_mps
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 3.915 | 2.064 | -47.3% | 1.497 | -61.8% |
| autoferry_scenario13_anchored | 1.068 | 1.267 | +18.6% | 1.185 | +11.0% |
| autoferry_scenario16_anchored | 0.987 | 0.994 | +0.7% | 0.922 | -6.6% |
| autoferry_scenario17_anchored | 0.746 | 0.800 | +7.2% | 0.818 | +9.7% |
| autoferry_scenario22_anchored | 1.082 | 1.370 | +26.6% | 1.264 | +16.8% |
| autoferry_scenario2_anchored | 1.062 | 1.130 | +6.4% | 1.145 | +7.8% |
| autoferry_scenario3_anchored | 0.624 | 0.740 | +18.6% | 0.739 | +18.5% |
| autoferry_scenario4_anchored | 1.226 | 1.042 | -15.0% | 1.039 | -15.3% |
| autoferry_scenario5_anchored | 1.349 | 1.314 | -2.6% | 1.312 | -2.7% |
| autoferry_scenario6_anchored | 1.202 | 1.066 | -11.4% | 1.113 | -7.4% |
| autoferry_scenario13 | 4.774 | 2.156 | -54.8% | 2.393 | -49.9% |
| autoferry_scenario16 | 4.186 | 1.302 | -68.9% | 4.613 | +10.2% |
| autoferry_scenario17 | 7.386 | 1.818 | -75.4% | 2.301 | -68.9% |
| autoferry_scenario2 | 3.230 | 1.540 | -52.3% | 1.456 | -54.9% |
| autoferry_scenario22 | 6.622 | 2.501 | -62.2% | 2.427 | -63.4% |
| autoferry_scenario3 | 6.139 | 1.773 | -71.1% | 1.652 | -73.1% |
| autoferry_scenario4 | 4.263 | 1.377 | -67.7% | 3.555 | -16.6% |
| autoferry_scenario5 | 5.957 | 4.409 | -26.0% | 2.486 | -58.3% |
| autoferry_scenario6 | 7.001 | 2.078 | -70.3% | 1.442 | -79.4% |
| dense_clutter | 2.447 | 4.578 | +87.1% | 3.741 | +52.9% |
| non_cooperative | 1.122 | 1.122 | 0.0% | 1.167 | +4.0% |
| ais_dropout | 3.095 | 5.665 | +83.0% | 5.281 | +70.7% |
| clock_skew | 1.389 | 2.100 | +51.2% | 2.100 | +51.2% |
| crossing | 2.845 | 4.851 | +70.5% | 4.851 | +70.5% |
| crossing_dropout | 2.852 | 4.855 | +70.2% | 4.855 | +70.2% |
| head_on | 2.845 | 4.851 | +70.5% | 4.850 | +70.5% |
| overtaking | 1.380 | 2.392 | +73.4% | 2.392 | +73.4% |
| parallel_targets | 1.501 | 1.749 | +16.6% | 1.749 | +16.6% |
| speed_change | 2.652 | 2.908 | +9.7% | 2.908 | +9.7% |

### nees_mean
| scenario | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|
| philos | 1.67 | 2.22 | +32.6% | 0.49 | -70.7% |
| autoferry_scenario13_anchored | 66.18 | 1.27 | -98.1% | 1.53 | -97.7% |
| autoferry_scenario16_anchored | 2.68 | 2.94 | +9.5% | 18.30 | +582.0% |
| autoferry_scenario17_anchored | 1.19 | 0.42 | -64.2% | 1.03 | -13.3% |
| autoferry_scenario22_anchored | 1.04 | 5.45 | +425.5% | 5.17 | +398.9% |
| autoferry_scenario2_anchored | 2.28 | 5.86 | +157.1% | 6.54 | +187.2% |
| autoferry_scenario3_anchored | 0.93 | 0.99 | +6.5% | 1.00 | +8.3% |
| autoferry_scenario4_anchored | 3.61 | 0.93 | -74.3% | 1.01 | -72.0% |
| autoferry_scenario5_anchored | 3.92 | 2.42 | -38.3% | 2.51 | -36.1% |
| autoferry_scenario6_anchored | 4.42 | 6.42 | +45.2% | 9.34 | +111.4% |
| autoferry_scenario13 | 51.68 | 12.92 | -75.0% | 12.05 | -76.7% |
| autoferry_scenario16 | 11.46 | 99.11 | +765.2% | 97.77 | +753.5% |
| autoferry_scenario17 | 67.32 | 6.80 | -89.9% | 12.61 | -81.3% |
| autoferry_scenario2 | 23.24 | 15.00 | -35.5% | 24.34 | +4.7% |
| autoferry_scenario22 | 216.23 | 331.64 | +53.4% | 334.51 | +54.7% |
| autoferry_scenario3 | 72.04 | 16.46 | -77.2% | 14.76 | -79.5% |
| autoferry_scenario4 | 57.52 | 5.09 | -91.2% | 4.66 | -91.9% |
| autoferry_scenario5 | 47.83 | 49.61 | +3.7% | 32.90 | -31.2% |
| autoferry_scenario6 | 75.15 | 68.16 | -9.3% | 81.94 | +9.0% |
| dense_clutter | 1.75 | 1.86 | +6.1% | 1.99 | +13.3% |
| non_cooperative | 0.15 | 0.15 | 0.0% | 0.14 | -4.4% |
| ais_dropout | 14.55 | 14.30 | -1.7% | 14.12 | -3.0% |
| clock_skew | 1.73 | 1.72 | -0.5% | 1.72 | -0.6% |
| crossing | 1.49 | 1.49 | -0.1% | 1.49 | -0.1% |
| crossing_dropout | 14.37 | 14.04 | -2.2% | 14.04 | -2.3% |
| head_on | 1.49 | 1.49 | -0.1% | 1.48 | -0.6% |
| overtaking | 1.43 | 1.43 | +0.0% | 1.43 | +0.0% |
| parallel_targets | 1.58 | 1.58 | -0.3% | 1.58 | -0.3% |
| speed_change | 2.29 | 2.27 | -1.0% | 2.26 | -1.1% |

## PMBM-only metrics (no MHT counterpart)
MHT does not emit a smoothed track output, so a delta vs MHT is meaningless. These metrics use the RTS smoother that the PMBM adapter wires.

### tgospa_smooth
| scenario | PMBM K=1 | PMBM K=3+xp |
|---|---:|---:|
| philos | 213.073 | 213.073 |
| autoferry_scenario13_anchored | 675.670 | 675.669 |
| autoferry_scenario16_anchored | 675.877 | 673.225 |
| autoferry_scenario17_anchored | 550.321 | 550.322 |
| autoferry_scenario22_anchored | 580.699 | 580.699 |
| autoferry_scenario2_anchored | 783.078 | 783.112 |
| autoferry_scenario3_anchored | 833.310 | 833.310 |
| autoferry_scenario4_anchored | 718.141 | 718.144 |
| autoferry_scenario5_anchored | 988.894 | 988.906 |
| autoferry_scenario6_anchored | 767.338 | 767.344 |
| autoferry_scenario13 | 678.823 | 678.823 |
| autoferry_scenario16 | 679.412 | 679.412 |
| autoferry_scenario17 | 557.494 | 557.494 |
| autoferry_scenario2 | 788.416 | 788.416 |
| autoferry_scenario22 | 584.466 | 584.466 |
| autoferry_scenario3 | 838.570 | 838.570 |
| autoferry_scenario4 | 724.431 | 724.431 |
| autoferry_scenario5 | 992.774 | 992.774 |
| autoferry_scenario6 | 772.269 | 772.269 |
| dense_clutter | 81.965 | 64.488 |
| non_cooperative | 114.142 | 114.142 |
| ais_dropout | 74.946 | 70.614 |
| clock_skew | 16.475 | 16.475 |
| crossing | 32.968 | 32.968 |
| crossing_dropout | 52.126 | 52.126 |
| head_on | 32.968 | 32.968 |
| overtaking | 67.081 | 67.081 |
| parallel_targets | 18.755 | 18.755 |
| speed_change | 21.012 | 21.012 |

## Per-class medians (seed-averaged, then median over scenarios in class)

### wall_seconds
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 33.446 | 23.610 | -29.4% | 357.132 | +967.8% |
| autoferry-anchored | 9 | 7.753 | 0.468 | -94.0% | 5.117 | -34.0% |
| autoferry-unanchored | 9 | 3.211 | 0.352 | -89.0% | 3.734 | +16.3% |
| stress / dense | 2 | 0.011 | 0.005 | -53.7% | 0.042 | +274.7% |
| synthetic | 8 | 0.003 | 0.002 | -26.9% | 0.006 | +79.7% |

### gospa_mean
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 69.427 | 82.627 | +19.0% | 82.454 | +18.8% |
| autoferry-anchored | 9 | 2.641 | 2.652 | +0.5% | 3.213 | +21.7% |
| autoferry-unanchored | 9 | 31.939 | 17.279 | -45.9% | 17.302 | -45.8% |
| stress / dense | 2 | 15.112 | 15.300 | +1.2% | 15.200 | +0.6% |
| synthetic | 8 | 8.222 | 7.931 | -3.5% | 7.923 | -3.6% |

### ospa_mean
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 424.611 | 454.804 | +7.1% | 454.611 | +7.1% |
| autoferry-anchored | 9 | 20.117 | 13.945 | -30.7% | 15.890 | -21.0% |
| autoferry-unanchored | 9 | 381.990 | 243.602 | -36.2% | 260.696 | -31.8% |
| stress / dense | 2 | 230.686 | 233.846 | +1.4% | 237.303 | +2.9% |
| synthetic | 8 | 19.039 | 5.954 | -68.7% | 5.948 | -68.8% |

### id_switches
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 0.0 | 0.1 | — | 0.1 | — |
| autoferry-anchored | 9 | 3.0 | 0.0 | -100.0% | 0.0 | -100.0% |
| autoferry-unanchored | 9 | 36.0 | 5.5 | -84.7% | 4.5 | -87.5% |
| stress / dense | 2 | 0.4 | 0.0 | -100.0% | 0.1 | -80.0% |
| synthetic | 8 | 0.0 | 0.0 | 0.0% | 0.0 | 0.0% |

### pos_rmse_m
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 25.583 | 12.901 | -49.6% | 10.796 | -57.8% |
| autoferry-anchored | 9 | 1.637 | 1.450 | -11.4% | 1.747 | +6.7% |
| autoferry-unanchored | 9 | 21.051 | 13.440 | -36.2% | 13.888 | -34.0% |
| stress / dense | 2 | 11.773 | 12.000 | +1.9% | 12.577 | +6.8% |
| synthetic | 8 | 6.509 | 6.545 | +0.6% | 6.540 | +0.5% |

### sog_rmse_mps
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 3.915 | 2.064 | -47.3% | 1.497 | -61.8% |
| autoferry-anchored | 9 | 1.068 | 1.066 | -0.2% | 1.113 | +4.2% |
| autoferry-unanchored | 9 | 5.957 | 1.818 | -69.5% | 2.393 | -59.8% |
| stress / dense | 2 | 1.784 | 2.850 | +59.7% | 2.454 | +37.5% |
| synthetic | 8 | 2.749 | 3.880 | +41.1% | 3.879 | +41.1% |

### nees_mean
| class | n | MHT | PMBM K=1 | Δ K=1 | PMBM K=3+xp | Δ K=3 |
|---|---:|---:|---:|---:|---:|---:|
| real-world (philos) | 1 | 1.67 | 2.22 | +32.6% | 0.49 | -70.7% |
| autoferry-anchored | 9 | 2.68 | 2.42 | -9.8% | 2.51 | -6.6% |
| autoferry-unanchored | 9 | 57.52 | 16.46 | -71.4% | 24.34 | -57.7% |
| stress / dense | 2 | 0.95 | 1.00 | +5.6% | 1.06 | +12.0% |
| synthetic | 8 | 1.66 | 1.65 | -0.4% | 1.65 | -0.4% |

## Win/loss tally vs MHT canonical (per metric)
Win = PMBM variant beats MHT canonical on this scenario (seed-mean). Threshold for 'tie': |Δ| < 1%.

| metric | K=1 wins | K=1 ties | K=1 losses | K=3 wins | K=3 ties | K=3 losses |
|---|---:|---:|---:|---:|---:|---:|
| wall_seconds | 28 | 0 | 1 | 9 | 0 | 20 |
| gospa_mean | 19 | 2 | 8 | 19 | 2 | 8 |
| ospa_mean | 19 | 1 | 9 | 19 | 2 | 8 |
| id_switches | 19 | 9 | 0 | 18 | 10 | 0 |
| pos_rmse_m | 12 | 9 | 8 | 11 | 8 | 10 |
| sog_rmse_mps | 13 | 2 | 14 | 13 | 0 | 16 |
| nees_mean | 14 | 5 | 10 | 16 | 4 | 9 |
