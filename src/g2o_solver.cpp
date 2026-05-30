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

class VertexPose2D : public g2o::BaseVertex<3, Pose2D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = Pose2D{}; }

    void oplusImpl(const double* update) override {
        _estimate.x     += update[0];
        _estimate.y     += update[1];
        _estimate.theta  = normaliseAngle(_estimate.theta + update[2]);
    }

    bool read(std::istream&)        override { return true; }
    bool write(std::ostream&) const override { return true; }
};

class EdgePose2D
    : public g2o::BaseBinaryEdge<3, Pose2D, VertexPose2D, VertexPose2D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override {
        const auto* vi = static_cast<const VertexPose2D*>(_vertices[0]);
        const auto* vj = static_cast<const VertexPose2D*>(_vertices[1]);

        Pose2D z_hat = vi->estimate().relativeTo(vj->estimate());

        _error[0] = _measurement.x     - z_hat.x;
        _error[1] = _measurement.y     - z_hat.y;
        _error[2] = normaliseAngle(_measurement.theta - z_hat.theta);
        // g2o applies _information to _error internally for chi2 and gradient
    }

    bool read(std::istream&)        override { return true; }
    bool write(std::ostream&) const override { return true; }
};

SolverResult solveG2O(PoseGraph& graph, int max_iter, bool verbose) {
    SolverResult result;
    result.solver_name  = "g2o";
    result.initial_cost = graph.totalCost();

    using BlockSolverType  = g2o::BlockSolverPL<3, 3>;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver  = std::make_unique<BlockSolverType>(std::move(linearSolver));
    auto* algorithm   = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(verbose);

    for (int i = 0; i < graph.numPoses(); ++i) {
        auto* v = new VertexPose2D();
        v->setId(i);
        v->setEstimate(graph.poses[i]);
        if (i == 0) v->setFixed(true);
        optimizer.addVertex(v);
    }

    for (int k = 0; k < graph.numEdges(); ++k) {
        const auto& e   = graph.edges[k];
        auto*       edge = new EdgePose2D();
        edge->setId(k);
        edge->setVertex(0, optimizer.vertex(e.from));
        edge->setVertex(1, optimizer.vertex(e.to));
        edge->setMeasurement(e.measurement);
        edge->setInformation(e.information);
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    auto t0   = std::chrono::steady_clock::now();
    int  iters = optimizer.optimize(max_iter);
    auto t1   = std::chrono::steady_clock::now();

    for (int i = 0; i < graph.numPoses(); ++i) {
        auto* v = dynamic_cast<VertexPose2D*>(optimizer.vertex(i));
        graph.poses[i] = v->estimate();
    }

    result.final_cost = graph.totalCost();
    result.iterations = iters;
    result.time_ms    = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.converged  = (iters > 0);

    return result;
}

}  // namespace slam_opt
