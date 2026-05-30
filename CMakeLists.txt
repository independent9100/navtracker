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
  tests/geo/test_wgs84.cpp
  tests/geo/test_datum.cpp
)
target_include_directories(navtracker_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(navtracker_tests PRIVATE navtracker_core GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(navtracker_tests)
