#include "g2o_solver.hpp"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <chrono>
#include <iostream>

namespace slam_opt {

// ================================================================
//  TODO  Milestone 3a: Implement VertexPose2D
// ================================================================

class VertexPose2D : public g2o::BaseVertex<3, Pose2D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override {
        _estimate = Pose2D{};
    }

    void oplusImpl(const double* update) override {
        // update is a 3-vector [dx, dy, dtheta]
        // Add it to _estimate; remember to normalise theta!
        _estimate.x += update[0];
        _estimate.y += update[1];
        _estimate.theta = normaliseAngle(_estimate.theta + update[2]);
    }

    // Serialisation (required; can be no-ops for this exercise)
    bool read(std::istream&)  override { return true; }
    bool write(std::ostream&) const override { return true; }
};

// ================================================================
//  TODO  Milestone 3b: Implement EdgePose2D
// ================================================================

class EdgePose2D
    : public g2o::BaseBinaryEdge<3, Pose2D, VertexPose2D, VertexPose2D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {
        const auto* vi = static_cast<const VertexPose2D*>(_vertices[0]);
        const auto* vj = static_cast<const VertexPose2D*>(_vertices[1]);

        // predicted measurement:  z_hat = pose_i^{-1} ⊕ pose_j
        Pose2D z_hat = vi->estimate().relativeTo(vj->estimate());

        // raw error
        _error[0] = _measurement.x     - z_hat.x;
        _error[1] = _measurement.y     - z_hat.y;
        _error[2] = normaliseAngle(_measurement.theta - z_hat.theta);

        // Note: g2o multiplies _error by _information internally when computing
        // the chi2 and the gradient, so you don't whiten here.
    }

    // Serialisation
    bool read(std::istream&)  override { return true; }
    bool write(std::ostream&) const override { return true; }
};

// ================================================================

SolverResult solveG2O(PoseGraph& graph, int max_iter, bool verbose) {
    SolverResult result;
    result.solver_name = "g2o";
    result.initial_cost = graph.totalCost();

    // TODO: Build the sparse optimizer
      using BlockSolverType = g2o::BlockSolverPL<3, 3>;
      using LinearSolverType =
          g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

      auto linearSolver = std::make_unique<LinearSolverType>();
      auto blockSolver  = std::make_unique<BlockSolverType>(
                              std::move(linearSolver));
      auto* algorithm   = new g2o::OptimizationAlgorithmGaussNewton(
                              std::move(blockSolver));

      g2o::SparseOptimizer optimizer;
      optimizer.setAlgorithm(algorithm);
      optimizer.setVerbose(verbose);

    // TODO: Add vertices
    for (int i = 0; i < graph.numPoses(); ++i) {
        auto* v = new VertexPose2D();
        v->setId(i);
        v->setEstimate(graph.poses[i]);
        if (i == 0) v->setFixed(true);
        optimizer.addVertex(v);
    }

    // TODO: Add edges
    for (int k = 0; k < graph.numEdges(); ++k) {
        const auto& e = graph.edges[k];
        auto* edge = new EdgePose2D();
        edge->setId(k);
        edge->setVertex(0, optimizer.vertex(e.from));
        edge->setVertex(1, optimizer.vertex(e.to));
        edge->setMeasurement(e.measurement);
        edge->setInformation(e.information);
        optimizer.addEdge(edge);
    }

    // TODO: Run optimisation and write results back
    optimizer.initializeOptimization();
    auto t0 = std::chrono::steady_clock::now();
    int iters = optimizer.optimize(max_iter);
    auto t1 = std::chrono::steady_clock::now();

    for (int i = 0; i < graph.numPoses(); ++i) {
        auto* v = dynamic_cast<VertexPose2D*>(optimizer.vertex(i));
        graph.poses[i] = v->estimate();
    }

    result.final_cost  = graph.totalCost();
    result.iterations  = iters;
    result.time_ms     = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.converged   = (iters > 0);

    return result;
}

}  // namespace slam_opt
