cmake_minimum_required(VERSION 3.16)
project(slam_opt_compare VERSION 1.0.0 LANGUAGES CXX)

# ============================================================
# Global settings
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # for clangd / IDE support

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
# Dependencies
# ============================================================

# --- Eigen (header-only, required by both Ceres and g2o) ---
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

# --- Ceres Solver ---
find_package(Ceres REQUIRED)
message(STATUS "Ceres found: ${CERES_VERSION}")

# --- g2o ---
# g2o does not always ship a proper CMake config; handle both cases.
find_package(g2o QUIET)
if(g2o_FOUND)
  message(STATUS "g2o found via find_package (version: ${g2o_VERSION})")
  set(G2O_LIBS
    g2o::core
    g2o::stuff
    g2o::types_slam2d
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
  )
else()
  # Fallback: try to locate g2o headers and shared libs manually
  message(STATUS "g2o CMake config not found — trying manual search")

  find_path(G2O_INCLUDE_DIR
    NAMES g2o/core/base_vertex.h
    PATHS /usr/include /usr/local/include
  )
  find_library(G2O_CORE_LIB    NAMES g2o_core    PATHS /usr/lib /usr/local/lib)
  find_library(G2O_STUFF_LIB   NAMES g2o_stuff   PATHS /usr/lib /usr/local/lib)
  find_library(G2O_TYPES_SLAM2D_LIB
                               NAMES g2o_types_slam2d
                               PATHS /usr/lib /usr/local/lib)
  find_library(G2O_SOLVER_EIGEN_LIB
                               NAMES g2o_solver_eigen
                               PATHS /usr/lib /usr/local/lib)
  find_library(G2O_SOLVER_CSPARSE_LIB
                               NAMES g2o_solver_csparse
                               PATHS /usr/lib /usr/local/lib)
  find_library(G2O_CSPARSE_EXT_LIB
                               NAMES g2o_csparse_extension
                               PATHS /usr/lib /usr/local/lib)

  if(NOT G2O_CORE_LIB OR NOT G2O_INCLUDE_DIR)
    message(FATAL_ERROR
      "g2o not found. Install with:  sudo apt install libg2o-dev")
  endif()

  message(STATUS "g2o (manual): ${G2O_INCLUDE_DIR}")
  include_directories(${G2O_INCLUDE_DIR})

  set(G2O_LIBS
    ${G2O_CORE_LIB}
    ${G2O_STUFF_LIB}
    ${G2O_TYPES_SLAM2D_LIB}
    ${G2O_SOLVER_EIGEN_LIB}
    ${G2O_SOLVER_CSPARSE_LIB}
    ${G2O_CSPARSE_EXT_LIB}
  )
endif()

# ============================================================
# Source files
# ============================================================
set(SOURCES
  src/main.cpp
  src/pose_graph.cpp
  src/ceres_solver.cpp
  src/g2o_solver.cpp
)

# ============================================================
# Executable
# ============================================================
add_executable(slam_opt_compare ${SOURCES})

target_include_directories(slam_opt_compare
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(slam_opt_compare
  PRIVATE
    Ceres::ceres
    ${G2O_LIBS}
    Eigen3::Eigen
)

# Enable strict warnings in debug builds
target_compile_options(slam_opt_compare PRIVATE
  $<$<CONFIG:Debug>:-Wall -Wextra -Wpedantic>
  $<$<CONFIG:Release>:-O3 -DNDEBUG>
)

# ============================================================
# Optional: unit tests (enable with -DBUILD_TESTS=ON)
# ============================================================
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(test_pose_math tests/test_pose_math.cpp src/pose_graph.cpp)
  target_include_directories(test_pose_math PRIVATE include)
  target_link_libraries(test_pose_math PRIVATE GTest::gtest_main Eigen3::Eigen)
  add_test(NAME pose_math COMMAND test_pose_math)
endif()

# ============================================================
# Print summary
# ============================================================
message(STATUS "")
message(STATUS "=== slam-opt-compare build summary ===")
message(STATUS "  CXX standard  : ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type    : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Ceres version : ${CERES_VERSION}")
message(STATUS "  Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "======================================")
