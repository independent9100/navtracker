add_executable(navtracker_bench_baseline baseline_matrix.cpp)
target_link_libraries(navtracker_bench_baseline
  PRIVATE navtracker_benchmark navtracker_benchmark_adapters)

# W6.3.2 — baseline provenance: capture the git sha (+ clean/dirty over TRACKED
# files) at configure time so every NEW baseline CSV records the commit that
# produced it. Best-effort: falls back to "unknown" if git is unavailable or
# this is not a checkout. Captured at configure time, so reconfigure (cmake)
# before generating a baseline you want stamped with a fresh sha.
find_package(Git QUIET)
set(_nav_git_sha "unknown")
if(Git_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE _nav_sha OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _nav_rc)
  if(_nav_rc EQUAL 0 AND NOT _nav_sha STREQUAL "")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE _nav_dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(_nav_dirty STREQUAL "")
      set(_nav_git_sha "${_nav_sha} (clean)")
    else()
      set(_nav_git_sha "${_nav_sha} (dirty)")
    endif()
  endif()
endif()
target_compile_definitions(navtracker_bench_baseline
  PRIVATE NAVTRACKER_GIT_SHA="${_nav_git_sha}")

add_executable(navtracker_bench_render render_markdown.cpp)
target_link_libraries(navtracker_bench_render
  PRIVATE navtracker_benchmark)

add_executable(navtracker_bench_compare compare.cpp)
target_link_libraries(navtracker_bench_compare PRIVATE navtracker_benchmark)

# Cl-4 Phase-1 A3 evidence probe — research tooling, measurement only.
add_executable(navtracker_cl4_a3_census ${CMAKE_SOURCE_DIR}/tools/cl4_a3_census.cpp)
target_link_libraries(navtracker_cl4_a3_census
  PRIVATE navtracker_benchmark navtracker_benchmark_adapters)
