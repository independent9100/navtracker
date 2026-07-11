add_executable(navtracker_bench_baseline baseline_matrix.cpp)
target_link_libraries(navtracker_bench_baseline
  PRIVATE navtracker_benchmark navtracker_benchmark_adapters)

add_executable(navtracker_bench_render render_markdown.cpp)
target_link_libraries(navtracker_bench_render
  PRIVATE navtracker_benchmark)

add_executable(navtracker_bench_compare compare.cpp)
target_link_libraries(navtracker_bench_compare PRIVATE navtracker_benchmark)

# Cl-4 Phase-1 A3 evidence probe — research tooling, measurement only.
add_executable(navtracker_cl4_a3_census ${CMAKE_SOURCE_DIR}/tools/cl4_a3_census.cpp)
target_link_libraries(navtracker_cl4_a3_census
  PRIVATE navtracker_benchmark navtracker_benchmark_adapters)
