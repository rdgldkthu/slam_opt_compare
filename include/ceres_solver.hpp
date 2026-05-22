#pragma once

#include "pose_graph.hpp"

namespace slam_opt {

// ----------------------------------------------------------------
// Ceres-based pose graph optimizer
//
// IMPLEMENTATION TASK (ceres_solver.cpp):
//
//  1. Define a cost functor struct `PoseGraphCostFunctor` that
//     encodes one Edge2D residual.
//     - Constructor takes a const reference to an Edge2D.
//     - operator() computes the 3-D whitened residual.
//     - Think: AutoDiff or Analytic Jacobian?
//
//  2. In solveCeres():
//     - Build a ceres::Problem.
//     - For each edge, create a CostFunction wrapping your functor
//       and add it to the problem.
//     - Fix the anchor pose (index 0) so the solution is unique.
//     - Run the solver and fill in the SolverResult.
//
//  Hints:
//    - Parameter block layout: double pose[3] = {x, y, theta}
//    - Whitened residual: r = L * e   where  Omega = L^T L  (Cholesky)
//    - ceres::Problem::SetParameterBlockConstant() for the anchor
// ----------------------------------------------------------------
SolverResult solveCeres(PoseGraph& graph,
                        bool       verbose = false);

}  // namespace slam_opt
