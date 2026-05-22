#pragma once

#include "pose_graph.hpp"

namespace slam_opt {

// ----------------------------------------------------------------
// g2o-based pose graph optimizer
//
// IMPLEMENTATION TASK (g2o_solver.cpp):
//
//  1. Define `VertexPose2D`:
//     - Inherit from g2o::BaseVertex<3, slam_opt::Pose2D>
//     - Implement setToOriginImpl() and oplusImpl()
//       * oplusImpl adds the update vector; remember angle wrap!
//
//  2. Define `EdgePose2D`:
//     - Inherit from g2o::BaseBinaryEdge<3, slam_opt::Pose2D,
//                                         VertexPose2D, VertexPose2D>
//     - Implement computeError()
//       * _error = measurement ^ (-1) composed with predicted z_hat
//       * Weight by the square-root information (g2o uses
//         _information internally — just set it from edge.information)
//     - Optionally implement linearizeOplus() for analytic Jacobians
//
//  3. In solveG2O():
//     - Create a SparseOptimizer, attach a linear solver and
//       an optimisation algorithm (e.g. Gauss-Newton or LM).
//     - Add vertices and edges from the PoseGraph.
//     - Fix vertex 0 (setFixed(true)).
//     - Call optimizer.optimize(max_iter).
//     - Write results back to graph.poses and fill SolverResult.
//
//  Hints:
//    - Typical solver chain:
//        auto linearSolver = std::make_unique<
//            g2o::LinearSolverEigen<g2o::BlockSolver_3_3::PoseMatrixType>>();
//        auto blockSolver  = std::make_unique<g2o::BlockSolver_3_3>(
//                                std::move(linearSolver));
//        auto algorithm    = new g2o::OptimizationAlgorithmGaussNewton(
//                                std::move(blockSolver));
//    - g2o::EdgeSE2 / g2o::VertexSE2 exist in the library; look at them
//      as reference, but write your own types from scratch.
// ----------------------------------------------------------------
SolverResult solveG2O(PoseGraph& graph,
                      int        max_iter = 100,
                      bool       verbose  = false);

}  // namespace slam_opt
