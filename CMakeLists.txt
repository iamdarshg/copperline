cmake_minimum_required(VERSION 3.20)
project(copperline VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4)
else()
  # Note: -Wpedantic is intentionally off; the core uses __int128 (GCC/Clang)
  # for exact integer clearance math. See README portability notes.
  add_compile_options(-Wall -Wextra)
endif()

add_library(router_core
  src/json.cpp
  src/sexpr.cpp
  src/board.cpp
  src/kicad.cpp
  src/kicad_export.cpp
  src/dsn.cpp
  src/gerber.cpp
  src/ipc2581.cpp
  src/rules.cpp
  src/spatial_index.cpp
  src/density.cpp
  src/route_tree.cpp
  src/sparse_graph.cpp
  src/astar.cpp
  src/hierarchy.cpp
  src/maturity.cpp
  src/simplify.cpp
  src/tuning.cpp
  src/via_bundle.cpp
  src/diffpair.cpp
  src/escape.cpp
  src/parallel.cpp
  src/portfolio.cpp
  src/impact.cpp
  src/recovery.cpp
  src/optimizer.cpp
  src/engine.cpp
  src/verifier.cpp
  src/analyze.cpp
)
target_include_directories(router_core PUBLIC include)
find_package(Threads REQUIRED)
target_link_libraries(router_core PUBLIC Threads::Threads)

add_executable(router src/main.cpp)
target_link_libraries(router PRIVATE router_core)
target_compile_definitions(router PRIVATE COPPERLINE_VERSION="${PROJECT_VERSION}")

enable_testing()

function(copperline_test name)
  add_executable(${name} tests/${name}.cpp tests/helpers.h)
  target_link_libraries(${name} PRIVATE router_core)
  target_include_directories(${name} PRIVATE tests)
  target_compile_definitions(${name} PRIVATE
    ROUTER_BIN="$<TARGET_FILE:router>"
    FIXTURE_DIR="${CMAKE_SOURCE_DIR}/fixtures")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

copperline_test(test_geometry)
copperline_test(test_hierarchy)
copperline_test(test_maturity)
copperline_test(test_simplify)
copperline_test(test_current)
copperline_test(test_impedance)
copperline_test(test_via_bundle)
copperline_test(test_ampacity)
copperline_test(test_voltage)
copperline_test(test_density)
copperline_test(test_engine)
copperline_test(test_route_tree_growth)
copperline_test(test_verifier)
copperline_test(test_escape)
copperline_test(test_parallel)
copperline_test(test_portfolio)
copperline_test(test_impact)
copperline_test(test_plane)
copperline_test(test_recovery)
copperline_test(test_multiply)
copperline_test(test_perf)
copperline_test(test_cli)
copperline_test(test_diffpair)
copperline_test(test_pair_verify)
copperline_test(test_tuning)
copperline_test(test_golden)
copperline_test(test_formats)
