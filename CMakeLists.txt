cmake_minimum_required(VERSION 3.20)
project(navtracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

enable_testing()

add_library(navtracker_core
  core/geo/Wgs84.cpp
  core/geo/Datum.cpp
  core/estimation/ConstantVelocity2D.cpp
  core/estimation/ConstantVelocity5State.cpp
  core/estimation/CoordinatedTurn.cpp
  core/estimation/PrescribedTurn.cpp
  core/estimation/ImmEstimator.cpp
  core/estimation/MeasurementModels.cpp
  core/estimation/EkfEstimator.cpp
  core/estimation/SigmaPoints.cpp
  core/estimation/UkfEstimator.cpp
  core/estimation/Resampling.cpp
  core/estimation/ParticleFilterEstimator.cpp
  core/bias/HeadingBiasEstimator.cpp
  core/bias/AisArpaPairExtractor.cpp
  core/own_ship/UereEstimator.cpp
  core/collision/Cpa.cpp
  core/association/Gating.cpp
  core/association/GnnAssociator.cpp
  core/association/JointEvents.cpp
  core/association/JpdaAssociator.cpp
  core/tracking/TrackManager.cpp
  core/tracking/TrackTree.cpp
  core/pipeline/ReorderBuffer.cpp
  core/pipeline/Tracker.cpp
  core/pipeline/MhtTracker.cpp
  adapters/own_ship/OwnShipProvider.cpp
  adapters/util/Projection.cpp
  adapters/util/Nmea.cpp
  adapters/ais/AisAdapter.cpp
  adapters/own_ship/OwnShipNmeaAdapter.cpp
  adapters/arpa/ArpaAdapter.cpp
  adapters/eoir/EoIrAdapter.cpp
  core/scenario/Builders.cpp
  core/scenario/Ospa.cpp
  core/scenario/Harness.cpp
  core/scenario/HarnessBatched.cpp
  core/scenario/HarnessBatchedMht.cpp
  core/scenario/Metrics.cpp
  sim/TruthTrajectory.cpp
  sim/NmeaEncode.cpp
  sim/OwnShipEmitter.cpp
  sim/AisEmitter.cpp
  sim/ArpaEmitter.cpp
  sim/EoIrEmitter.cpp
  sim/SimulatedSensorBus.cpp
)
target_include_directories(navtracker_core PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_core PUBLIC Eigen3::Eigen)

add_executable(navtracker_tests
  tests/smoke_test.cpp
  tests/types/test_timestamp.cpp
  tests/types/test_ids.cpp
  tests/types/test_measurement.cpp
  tests/types/test_track.cpp
  tests/estimation/test_constant_velocity.cpp
  tests/estimation/test_constant_velocity_5state.cpp
  tests/estimation/test_coordinated_turn.cpp
  tests/estimation/test_prescribed_turn.cpp
  tests/estimation/test_imm_estimator.cpp
  tests/estimation/test_measurement_models.cpp
  tests/estimation/test_ekf_estimator.cpp
  tests/estimation/test_sigma_points.cpp
  tests/estimation/test_ukf_estimator.cpp
  tests/estimation/test_resampling.cpp
  tests/estimation/test_particle_filter_estimator.cpp
  tests/collision/test_cpa.cpp
  tests/association/test_gating.cpp
  tests/association/test_gnn_associator.cpp
  tests/association/test_joint_events.cpp
  tests/association/test_jpda_associator.cpp
  tests/tracking/test_track_manager.cpp
  tests/tracking/test_track_tree.cpp
  tests/pipeline/test_reorder_buffer.cpp
  tests/ports/test_edge_ports.cpp
  tests/pipeline/test_tracker.cpp
  tests/pipeline/test_mht_tracker.cpp
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
  tests/adapters/own_ship/test_own_ship_provider.cpp
  tests/adapters/util/test_projection.cpp
  tests/adapters/util/test_nmea.cpp
  tests/adapters/ais/test_ais_adapter.cpp
  tests/adapters/own_ship/test_own_ship_nmea.cpp
  tests/adapters/arpa/test_arpa_adapter.cpp
  tests/adapters/eoir/test_eoir_adapter.cpp
  tests/adapters/test_end_to_end.cpp
  tests/scenario/test_builders.cpp
  tests/scenario/test_ospa.cpp
  tests/scenario/test_harness.cpp
  tests/scenario/test_harness_batched.cpp
  tests/scenario/test_parallel_targets.cpp
  tests/scenario/test_metrics.cpp
  tests/scenario/test_crossing.cpp
  tests/scenario/test_overtaking.cpp
  tests/scenario/test_ais_dropout.cpp
  tests/scenario/test_filter_comparison.cpp
  tests/scenario/test_jpda_comparison.cpp
  tests/scenario/test_mht_comparison.cpp
  tests/scenario/test_multi_seed_sweep.cpp
  tests/scenario/test_truth_tick_ospa.cpp
  tests/sim/test_truth_trajectory.cpp
  tests/sim/test_maneuvering_trajectory.cpp
  tests/sim/test_nmea_encode.cpp
  tests/sim/test_own_ship_emitter.cpp
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
  tests/sim/test_bus_heading_bias_drift_probe.cpp
  tests/sim/test_bus_bias_estimator_sweep.cpp
  tests/sim/test_bus_anchor_loss.cpp
  tests/bias/test_heading_bias_estimator.cpp
  tests/bias/test_ais_arpa_pair_extractor.cpp
  tests/own_ship/test_uere_estimator.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
