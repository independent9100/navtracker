cmake_minimum_required(VERSION 3.20)
project(navtracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Emit compile_commands.json for clangd / IDE LSPs. Symlinked at the repo
# root (compile_commands.json -> build/compile_commands.json) so clangd
# finds it without extra config.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(nlohmann_json REQUIRED)

option(NAVTRACKER_BUILD_FOXGLOVE "Build the Foxglove/MCAP debug recorder adapter" ON)
if(NAVTRACKER_BUILD_FOXGLOVE)
  find_package(mcap REQUIRED)
endif()

enable_testing()

add_library(navtracker_core
  core/types/SensorDefaults.cpp
  core/types/MeasurementBuilders.cpp
  core/geo/Wgs84.cpp
  core/geo/Datum.cpp
  core/geo/AxisRotation.cpp
  core/projection/Projection.cpp
  core/estimation/ConstantVelocity2D.cpp
  core/estimation/ConstantVelocity5State.cpp
  core/estimation/CoordinatedTurn.cpp
  core/estimation/PrescribedTurn.cpp
  core/estimation/ImmEstimator.cpp
  core/estimation/MeasurementModels.cpp
  core/estimation/EstimatorDefaults.cpp
  core/estimation/EkfEstimator.cpp
  core/estimation/BearingRangeGuard.cpp
  core/estimation/SigmaPoints.cpp
  core/estimation/UkfEstimator.cpp
  core/estimation/Resampling.cpp
  core/estimation/ParticleFilterEstimator.cpp
  core/bias/HeadingBiasEstimator.cpp
  core/bias/AisArpaPairExtractor.cpp
  core/bias/SensorBiasEstimator.cpp
  core/bias/SensorBiasPairExtractor.cpp
  core/own_ship/UereEstimator.cpp
  core/own_ship/OwnShipVelocityEstimator.cpp
  core/own_ship/OwnShipProvider.cpp
  core/own_ship/NavInputGuard.cpp
  core/collision/Cpa.cpp
  core/collision/CpaOwnShip.cpp
  core/collision/CpaEvaluator.cpp
  core/collision/StaticHazardEvaluator.cpp
  core/association/Gating.cpp
  core/association/GnnAssociator.cpp
  core/association/JointEvents.cpp
  core/association/JpdaAssociator.cpp
  core/association/Hungarian.cpp
  core/association/Murty.cpp
  core/tracking/TrackManager.cpp
  core/tracking/TrackTree.cpp
  core/tracking/ClutterMapDetectionModel.cpp
  core/tracking/DatumShift.cpp
  core/pipeline/ReorderBuffer.cpp
  core/pipeline/Tracker.cpp
  core/pipeline/MhtTracker.cpp
  core/pmbm/PmbmTracker.cpp
  core/land/CoastlineGeometry.cpp
  core/scenario/Builders.cpp
  core/scenario/Ospa.cpp
  core/scenario/Gospa.cpp
  core/scenario/TGospa.cpp
  core/scenario/Harness.cpp
  core/scenario/HarnessBatched.cpp
  core/scenario/HarnessBatchedMht.cpp
  core/scenario/HarnessBatchedPmbm.cpp
  core/scenario/Metrics.cpp
  core/scenario/TruthResample.cpp
  core/output/TrackOutput.cpp
  core/output/StaticHazardOutput.cpp
  core/output/BearingWedgeOutput.cpp
  core/static/LiveOccupancyModel.cpp
  core/static/BearingWedgeModel.cpp
  core/sensor_activity/DeclaredSensorActivity.cpp
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)

# Track-to-track fusion (tracker-of-trackers). Consumer target: links the pure
# domain core, no I/O. See docs/algorithms/t2t-fusion.md.
add_library(navtracker_t2t
  core/t2t/ExternalTrack.cpp
  core/t2t/FusedTrackOutput.cpp
  core/t2t/T2tAssociator.cpp
  core/t2t/T2tFuser.cpp
)
target_include_directories(navtracker_t2t PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_t2t PUBLIC navtracker_core Eigen3::Eigen)

add_library(navtracker_nmea
  adapters/util/Nmea.cpp
  adapters/util/EdgeValidation.cpp
  adapters/ais/AisAdapter.cpp
  adapters/arpa/ArpaAdapter.cpp
  adapters/eoir/EoIrAdapter.cpp
  adapters/own_ship/OwnShipNmeaAdapter.cpp
  adapters/remote_track/RemoteTrackAdapter.cpp
)
target_include_directories(navtracker_nmea PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_nmea PUBLIC navtracker_core Eigen3::Eigen)

add_library(navtracker_replay
  adapters/replay/PlotCsvReplayAdapter.cpp
  adapters/replay/HaxrTruthLoader.cpp
  adapters/replay/AisCsvReplayAdapter.cpp
  adapters/replay/OwnshipCsvReader.cpp
  adapters/replay/AutoferryJsonReplay.cpp
  adapters/replay/RadarTruthCsvReader.cpp
  adapters/replay/CameraBearingCsvReader.cpp
)
target_include_directories(navtracker_replay PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_replay PUBLIC navtracker_core Eigen3::Eigen)

add_library(navtracker_land
  adapters/land/GeoJsonCoastline.cpp
  adapters/static/GeoJsonStaticObstacles.cpp
)
target_include_directories(navtracker_land PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_land PUBLIC navtracker_core Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)

add_library(navtracker_benchmark
  core/benchmark/Placeholder.hpp  # header-only at this stage
  core/benchmark/BenchSink.cpp
  core/benchmark/BenchRunner.cpp
  core/benchmark/Consistency.cpp
  core/benchmark/Metrics.cpp
  core/benchmark/Config.cpp
  core/benchmark/Sweep.cpp
  core/benchmark/GospaExport.cpp
  core/benchmark/CsvWriter.cpp
  core/benchmark/CsvReader.cpp
  core/benchmark/ExistenceLabel.cpp
  core/benchmark/MarkdownRenderer.cpp
  core/benchmark/Comparator.cpp
)
set_target_properties(navtracker_benchmark PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(navtracker_benchmark PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark PUBLIC navtracker_core navtracker_land Eigen3::Eigen)

add_library(navtracker_sim
  sim/TruthTrajectory.cpp
  sim/NmeaEncode.cpp
  sim/OwnShipEmitter.cpp
  sim/AisEmitter.cpp
  sim/ArpaEmitter.cpp
  sim/EoIrEmitter.cpp
  sim/SimulatedSensorBus.cpp
  sim/SkewInjector.cpp
)
target_include_directories(navtracker_sim PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_sim PUBLIC navtracker_nmea Eigen3::Eigen)

add_library(navtracker_benchmark_adapters
  adapters/benchmark/SimScenarioRun.cpp
  adapters/benchmark/ReplayScenarioRun.cpp
  adapters/benchmark/SimMultisensorScenarioRun.cpp
  adapters/benchmark/RbadScenarioRun.cpp
)
target_include_directories(navtracker_benchmark_adapters PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark_adapters
  PUBLIC navtracker_benchmark navtracker_core navtracker_sim navtracker_replay Eigen3::Eigen)

if(NAVTRACKER_BUILD_FOXGLOVE)
  add_library(navtracker_foxglove
    adapters/foxglove/Geometry.cpp
    adapters/foxglove/FoxgloveJson.cpp
    adapters/foxglove/McapWriter.cpp
    adapters/foxglove/FoxgloveDebugRecorder.cpp)
  target_include_directories(navtracker_foxglove PUBLIC ${CMAKE_SOURCE_DIR})
  target_link_libraries(navtracker_foxglove
    PUBLIC navtracker_core Eigen3::Eigen nlohmann_json::nlohmann_json
    PRIVATE mcap::mcap)
  target_compile_definitions(navtracker_foxglove PRIVATE
    NAVTRACKER_FOXGLOVE_SCHEMA_DIR="${CMAKE_SOURCE_DIR}/adapters/foxglove/schemas")
endif()

add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/types/test_ids.cpp
  tests/types/test_measurement.cpp
  tests/types/test_sensor_defaults.cpp
  tests/types/test_measurement_builders.cpp
  tests/types/test_track.cpp
  tests/estimation/test_constant_velocity.cpp
  tests/estimation/test_constant_velocity_5state.cpp
  tests/estimation/test_coordinated_turn.cpp
  tests/estimation/test_prescribed_turn.cpp
  tests/estimation/test_imm_estimator.cpp
  tests/estimation/test_measurement_models.cpp
  tests/estimation/test_ekf_estimator.cpp
  tests/estimation/test_bearing_range_guard.cpp
  tests/estimation/test_sigma_points.cpp
  tests/estimation/test_ukf_estimator.cpp
  tests/estimation/test_resampling.cpp
  tests/estimation/test_particle_filter_estimator.cpp
  tests/collision/test_cpa.cpp
  tests/collision/test_cpa_uncertainty.cpp
  tests/collision/test_cpa_synthesize_own_ship.cpp
  tests/collision/test_cpa_evaluator.cpp
  tests/collision/test_static_hazard_evaluator.cpp
  tests/association/test_gating.cpp
  tests/association/test_gnn_associator.cpp
  tests/association/test_joint_events.cpp
  tests/association/test_jpda_associator.cpp
  tests/association/test_hungarian.cpp
  tests/association/test_murty.cpp
  tests/tracking/test_track_manager.cpp
  tests/tracking/test_track_sink.cpp
  tests/tracking/test_track_tree.cpp
  tests/tracking/test_datum_shift.cpp
  tests/pipeline/test_reorder_buffer.cpp
  tests/ports/test_edge_ports.cpp
  tests/pipeline/test_source_touch_populate.cpp
  tests/pipeline/test_tracker.cpp
  tests/pipeline/test_tracker_bearing_innovation_emit.cpp
  tests/pipeline/test_tracker_innovation_emit.cpp
  tests/pipeline/test_mht_tracker.cpp
  tests/pipeline/test_mht_cross_merge.cpp
  tests/pipeline/test_mht_innovation_emit.cpp
  tests/pipeline/test_batch_ordering.cpp
  tests/pipeline/test_review_fixes.cpp
  tests/pipeline/test_jpda_contributing_sources.cpp
  tests/pipeline/test_bias_correction_orthogonality.cpp
  tests/land/test_coastline_geometry.cpp
  tests/land/test_coastline_model.cpp
  tests/land/test_geojson_coastline.cpp
  tests/pmbm/test_pmbm_types.cpp
  tests/pmbm/test_pmbm_tracker_predict.cpp
  tests/pmbm/test_pmbm_tracker_update.cpp
  tests/pmbm/test_pmbm_scenario.cpp
  tests/pmbm/test_pmbm_phase8.cpp
  tests/pmbm/test_pmbm_sensor_activity.cpp
  tests/pmbm/test_pmbm_remote_track_fusion.cpp
  tests/pmbm/test_pmbm_land_model.cpp
  tests/pmbm/test_cl4_near_shore_zone.cpp
  tests/pmbm/test_pmbm_static_obstacle.cpp
  tests/pmbm/test_pmbm_clutter_feed.cpp
  tests/pmbm/test_pmbm_birth_floor.cpp
  tests/pmbm/test_pmbm_pda_soft_update.cpp
  tests/static/test_static_obstacle_model.cpp
  tests/static/test_live_occupancy_model.cpp
  tests/static/test_bearing_wedge_model.cpp
  tests/static/test_shadow_mask.cpp
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
  tests/adapters/own_ship/test_own_ship_provider.cpp
  tests/adapters/util/test_projection.cpp
  tests/adapters/util/test_nmea.cpp
  tests/adapters/util/test_edge_validation.cpp
  tests/adapters/ais/test_ais_adapter.cpp
  tests/adapters/remote_track/test_remote_track_adapter.cpp
  tests/adapters/own_ship/test_own_ship_nmea.cpp
  tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp
  tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp
  tests/adapters/arpa/test_arpa_adapter.cpp
  tests/adapters/eoir/test_eoir_adapter.cpp
  tests/adapters/test_adapter_datum_recenter.cpp
  tests/adapters/test_end_to_end.cpp
  tests/adapters/static/test_geojson_static_obstacles.cpp
  tests/scenario/test_builders.cpp
  tests/scenario/test_ospa.cpp
  tests/scenario/test_gospa.cpp
  tests/scenario/test_tgospa.cpp
  tests/scenario/test_harness.cpp
  tests/scenario/test_harness_batched.cpp
  tests/scenario/test_parallel_targets.cpp
  tests/scenario/test_metrics.cpp
  tests/scenario/test_crossing.cpp
  tests/scenario/test_overtaking.cpp
  tests/scenario/test_ais_dropout.cpp
  tests/scenario/test_reorder_buffer_e2e.cpp
  tests/scenario/test_bearing_bias_convergence.cpp
  tests/integration/test_full_stack_pipeline.cpp
  tests/integration/test_static_obstacle_pipeline.cpp
  tests/integration/test_t2t_full_stack.cpp
  tests/scenario/test_filter_comparison.cpp
  tests/scenario/test_cpa_scenario.cpp
  tests/scenario/test_jpda_comparison.cpp
  tests/scenario/test_mht_comparison.cpp
  tests/scenario/test_multi_seed_sweep.cpp
  tests/scenario/test_truth_tick_ospa.cpp
  tests/scenario/test_truth_resample.cpp
  tests/scenario/test_datum_recenter_scenario.cpp
  tests/sim/test_truth_trajectory.cpp
  tests/sim/test_maneuvering_trajectory.cpp
  tests/sim/test_nmea_encode.cpp
  tests/sim/test_own_ship_emitter.cpp
  tests/sim/test_skew_injector.cpp
  tests/sim/test_ais_emitter.cpp
  tests/sim/test_arpa_emitter.cpp
  tests/sim/test_arpa_clutter.cpp
  tests/sim/test_eoir_emitter.cpp
  tests/sim/test_simulated_sensor_bus.cpp
  tests/sim/test_bus_determinism.cpp
  tests/sim/test_bus_regression.cpp
  tests/sim/test_per_window_ospa.cpp
  tests/sim/test_bus_jpda_comparison.cpp
  tests/sim/test_bus_imm3_comparison.cpp
  tests/sim/test_bus_pf_comparison.cpp
  tests/sim/test_bus_mht_comparison.cpp
  tests/sim/test_bus_heading_sweep.cpp
  tests/sim/test_bus_gps_sweep.cpp
  tests/sim/test_bus_adaptive_uere.cpp
  tests/sim/test_bus_heading_bias_drift_probe.cpp
  tests/sim/test_bus_bias_estimator_sweep.cpp
  tests/sim/test_bus_anchor_loss.cpp
  tests/sim/test_bus_cpa_uncertainty.cpp
  tests/bias/test_heading_bias_estimator.cpp
  tests/bias/test_heading_bias_sign_convention.cpp
  tests/bias/test_heading_bias_closed_loop.cpp
  tests/bias/test_heading_bias_bearing_innovation.cpp
  tests/bias/test_heading_bias_multi_source.cpp
  tests/bias/test_heading_bias_permutations.cpp
  tests/bias/test_ais_arpa_pair_extractor.cpp
  tests/bias/test_sensor_bias_estimator.cpp
  tests/bias/test_sensor_bias_closed_loop.cpp
  tests/bias/test_bias_off_datum.cpp
  tests/bias/test_bias_correction.cpp
  tests/own_ship/test_uere_estimator.cpp
  tests/own_ship/test_own_ship_velocity_estimator.cpp
  tests/own_ship/test_nav_input_guard.cpp
  tests/output/test_track_output.cpp
  tests/output/test_static_hazard_output.cpp
  tests/replay/test_haxr_ospa.cpp
  tests/replay/test_philos_ospa.cpp
  tests/replay/test_philos_farcross.cpp
  tests/replay/test_philos_sunset_labels.cpp
  tests/replay/test_philos_close_approach_labels.cpp
  tests/replay/test_philos_close_approach_cpa.cpp
  tests/replay/test_heldout_sailboats_probe.cpp
  tests/replay/test_philos_occupancy_coverage_6c.cpp
  tests/replay/test_philos_los_shadow_probe.cpp
  tests/replay/test_radar_truth_loader.cpp
  tests/replay/test_camera_bearing_loader.cpp
  tests/replay/test_ais_csv_replay_adapter.cpp
  tests/replay/test_autoferry_json_replay.cpp
  tests/estimation/test_noise_models.cpp
  tests/tracking/test_sensor_detection_models.cpp
  tests/tracking/test_clutter_map_detection_model.cpp
  tests/pipeline/test_mht_sprt_confirm.cpp
  tests/pipeline/test_mht_ipda_lifecycle.cpp
  tests/benchmark/test_smoke.cpp
  tests/benchmark/test_scenario_run_port.cpp
  tests/benchmark/test_bench_sink.cpp
  tests/benchmark/test_bench_runner.cpp
  tests/benchmark/test_metrics.cpp
  tests/benchmark/test_consistency.cpp
  tests/benchmark/test_config.cpp
  tests/benchmark/test_sim_scenario_run.cpp
  tests/benchmark/test_replay_scenario_run.cpp
  tests/benchmark/test_sim_multisensor_scenario_run.cpp
  tests/benchmark/test_imazu22_scenario_run.cpp
  tests/benchmark/test_rbad_scenario_run.cpp
  tests/benchmark/test_sweep.cpp
  tests/benchmark/test_csv_writer.cpp
  tests/benchmark/test_csv_reader.cpp
  tests/benchmark/test_render_markdown.cpp
  tests/benchmark/test_compare.cpp
  tests/benchmark/test_bench_determinism.cpp
  tests/benchmark/test_synthetic_clutter_ab.cpp
  tests/benchmark/test_philos_cluttermap_ab.cpp
  tests/benchmark/test_occupancy_ab.cpp
  tests/benchmark/test_los_guard_haxr_ab.cpp
  tests/benchmark/test_veto_isolation_haxr_ab.cpp
  tests/benchmark/test_veto_isolation_sim_ab.cpp
  tests/benchmark/test_harbor_complete_truth.cpp
  tests/benchmark/test_harbor_complete_truth_baseline.cpp
  tests/benchmark/test_adapt_k3_harbor_knife_guard.cpp
  tests/benchmark/test_clutter_burst_birth_confirm_probe.cpp
  tests/benchmark/test_gospa_export.cpp
  tests/benchmark/test_harbor_boat_near_pier.cpp
  tests/benchmark/test_harbor_gate_scenarios.cpp
  tests/benchmark/test_occupancy_detector_gates.cpp
  tests/benchmark/test_existence_label.cpp
  tests/sensor_activity/test_declared_sensor_activity.cpp
  tests/docs/test_integration_guide_config_coverage.cpp
  tests/t2t/test_covariance_intersection.cpp
  tests/t2t/test_pedigree.cpp
  tests/t2t/test_external_track_validation.cpp
  tests/t2t/test_associator.cpp
  tests/t2t/test_fuser.cpp
  tests/t2t/test_navtracker_source.cpp
  tests/benchmark/test_t2t_scenario_run.cpp
  tests/support/FixtureGuard.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE
    navtracker_core navtracker_nmea navtracker_sim navtracker_replay navtracker_benchmark
    navtracker_benchmark_adapters navtracker_land navtracker_t2t
    GTest::gtest_main Eigen3::Eigen)
target_compile_definitions(navtracker_tests PRIVATE
    NAVTRACKER_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(NAVTRACKER_BUILD_FOXGLOVE)
  target_sources(navtracker_tests PRIVATE
    tests/adapters/foxglove/test_geometry.cpp
    tests/adapters/foxglove/test_foxglove_json.cpp
    tests/adapters/foxglove/test_mcap_writer.cpp
    tests/adapters/foxglove/test_recorder.cpp
    tests/adapters/foxglove/test_recorder_determinism.cpp)
  target_link_libraries(navtracker_tests PRIVATE navtracker_foxglove mcap::mcap)
endif()

include(GoogleTest)
# W2.7: per-test TIMEOUT watchdog — a hung fixture-gated test otherwise stalls
# ctest with no bound. 300 s is far above the slowest real-data replay case
# while still catching a true hang. DISCOVERY_TIMEOUT guards the build-time
# --gtest_list_tests step (which loads the fixture env global) the same way.
gtest_discover_tests(navtracker_tests
    PROPERTIES TIMEOUT 300
    DISCOVERY_TIMEOUT 120)

add_executable(navtracker_core_only_smoke
    tests/types/test_link_core_only.cpp)
target_link_libraries(navtracker_core_only_smoke PRIVATE
    navtracker_core Eigen3::Eigen)

add_test(NAME CoreOnlyLinks COMMAND $<TARGET_FILE:navtracker_core_only_smoke>)

add_executable(navtracker_example app/example.cpp)
target_link_libraries(navtracker_example PRIVATE
    navtracker_core Eigen3::Eigen)

add_executable(navtracker_mht_fusion_example app/mht_fusion_example.cpp)
target_link_libraries(navtracker_mht_fusion_example PRIVATE
    navtracker_core Eigen3::Eigen)

add_executable(navtracker_t2t_example app/example_t2t.cpp)
target_link_libraries(navtracker_t2t_example PRIVATE
    navtracker_t2t navtracker_core Eigen3::Eigen)

if(NAVTRACKER_BUILD_FOXGLOVE)
  add_executable(navtracker_example_foxglove app/example.cpp)
  target_compile_definitions(navtracker_example_foxglove PRIVATE NAVTRACKER_WITH_FOXGLOVE)
  target_link_libraries(navtracker_example_foxglove PRIVATE
    navtracker_core navtracker_foxglove Eigen3::Eigen)

  add_executable(navtracker_foxglove_autoferry tools/foxglove_autoferry.cpp)
  target_link_libraries(navtracker_foxglove_autoferry PRIVATE
    navtracker_core navtracker_replay navtracker_foxglove Eigen3::Eigen)

  # PMBM + static-world debug drawing demo: exercises the environment / PMBM /
  # occupancy / coverage layers end to end into one .mcap.
  add_executable(navtracker_foxglove_pmbm_scenario tools/foxglove_pmbm_scenario.cpp)
  target_link_libraries(navtracker_foxglove_pmbm_scenario PRIVATE
    navtracker_core navtracker_replay navtracker_foxglove Eigen3::Eigen)
endif()

add_subdirectory(bench)
