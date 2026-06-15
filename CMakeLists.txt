cmake_minimum_required(VERSION 3.20)
project(navtracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

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
  core/collision/Cpa.cpp
  core/collision/CpaOwnShip.cpp
  core/collision/CpaEvaluator.cpp
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
  core/scenario/Builders.cpp
  core/scenario/Ospa.cpp
  core/scenario/Gospa.cpp
  core/scenario/Harness.cpp
  core/scenario/HarnessBatched.cpp
  core/scenario/HarnessBatchedMht.cpp
  core/scenario/Metrics.cpp
  core/scenario/TruthResample.cpp
  core/output/TrackOutput.cpp
  adapters/own_ship/OwnShipProvider.cpp
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)

add_library(navtracker_nmea
  adapters/util/Nmea.cpp
  adapters/ais/AisAdapter.cpp
  adapters/arpa/ArpaAdapter.cpp
  adapters/eoir/EoIrAdapter.cpp
  adapters/own_ship/OwnShipNmeaAdapter.cpp
)
target_include_directories(navtracker_nmea PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_nmea PUBLIC navtracker_core Eigen3::Eigen)

add_library(navtracker_replay
  adapters/replay/PlotCsvReplayAdapter.cpp
  adapters/replay/HaxrTruthLoader.cpp
  adapters/replay/AisCsvReplayAdapter.cpp
  adapters/replay/OwnshipCsvReader.cpp
  adapters/replay/AutoferryJsonReplay.cpp
)
target_include_directories(navtracker_replay PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_replay PUBLIC navtracker_core Eigen3::Eigen)

add_library(navtracker_benchmark
  core/benchmark/Placeholder.hpp  # header-only at this stage
  core/benchmark/BenchSink.cpp
  core/benchmark/BenchRunner.cpp
  core/benchmark/Consistency.cpp
  core/benchmark/Metrics.cpp
  core/benchmark/Config.cpp
  core/benchmark/Sweep.cpp
  core/benchmark/CsvWriter.cpp
  core/benchmark/CsvReader.cpp
  core/benchmark/MarkdownRenderer.cpp
  core/benchmark/Comparator.cpp
)
set_target_properties(navtracker_benchmark PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(navtracker_benchmark PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark PUBLIC navtracker_core Eigen3::Eigen)

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
)
target_include_directories(navtracker_benchmark_adapters PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_benchmark_adapters
  PUBLIC navtracker_benchmark navtracker_core navtracker_sim navtracker_replay Eigen3::Eigen)

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
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
  tests/adapters/own_ship/test_own_ship_provider.cpp
  tests/adapters/util/test_projection.cpp
  tests/adapters/util/test_nmea.cpp
  tests/adapters/ais/test_ais_adapter.cpp
  tests/adapters/own_ship/test_own_ship_nmea.cpp
  tests/adapters/own_ship/test_own_ship_nmea_multi_heading.cpp
  tests/adapters/own_ship/test_own_ship_nmea_bias_dispatch.cpp
  tests/adapters/arpa/test_arpa_adapter.cpp
  tests/adapters/eoir/test_eoir_adapter.cpp
  tests/adapters/test_end_to_end.cpp
  tests/scenario/test_builders.cpp
  tests/scenario/test_ospa.cpp
  tests/scenario/test_gospa.cpp
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
  tests/bias/test_heading_bias_bearing_innovation.cpp
  tests/bias/test_heading_bias_multi_source.cpp
  tests/bias/test_heading_bias_permutations.cpp
  tests/bias/test_ais_arpa_pair_extractor.cpp
  tests/bias/test_sensor_bias_estimator.cpp
  tests/own_ship/test_uere_estimator.cpp
  tests/own_ship/test_own_ship_velocity_estimator.cpp
  tests/output/test_track_output.cpp
  tests/replay/test_haxr_ospa.cpp
  tests/replay/test_philos_ospa.cpp
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
  tests/benchmark/test_sweep.cpp
  tests/benchmark/test_csv_writer.cpp
  tests/benchmark/test_csv_reader.cpp
  tests/benchmark/test_render_markdown.cpp
  tests/benchmark/test_compare.cpp
  tests/benchmark/test_bench_determinism.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE
    navtracker_core navtracker_nmea navtracker_sim navtracker_replay navtracker_benchmark
    navtracker_benchmark_adapters
    GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)

add_executable(navtracker_core_only_smoke
    tests/types/test_link_core_only.cpp)
target_link_libraries(navtracker_core_only_smoke PRIVATE
    navtracker_core Eigen3::Eigen)

add_test(NAME CoreOnlyLinks COMMAND $<TARGET_FILE:navtracker_core_only_smoke>)

add_executable(navtracker_example app/example.cpp)
target_link_libraries(navtracker_example PRIVATE
    navtracker_core Eigen3::Eigen)

add_subdirectory(bench)
