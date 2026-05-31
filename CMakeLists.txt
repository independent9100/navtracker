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
  core/estimation/MeasurementModels.cpp
  core/estimation/EkfEstimator.cpp
  core/association/Gating.cpp
  core/association/GnnAssociator.cpp
  core/tracking/TrackManager.cpp
  core/pipeline/ReorderBuffer.cpp
  core/pipeline/Tracker.cpp
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
  tests/estimation/test_measurement_models.cpp
  tests/estimation/test_ekf_estimator.cpp
  tests/association/test_gating.cpp
  tests/association/test_gnn_associator.cpp
  tests/tracking/test_track_manager.cpp
  tests/pipeline/test_reorder_buffer.cpp
  tests/ports/test_edge_ports.cpp
  tests/pipeline/test_tracker.cpp
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
  tests/scenario/test_parallel_targets.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
