#include "ceres_solver.hpp"

#include <ceres/ceres.h>
#include <chrono>
#include <iostream>

namespace slam_opt {

struct PoseGraphCostFunctor {
    explicit PoseGraphCostFunctor(const Edge2D& edge) : edge_(edge) {
        L_ = edge.information.llt().matrixL();
    }

    template <typename T>
    bool operator()(const T* const pose_i,
                    const T* const pose_j,
                    T* residuals) const {
        const T sin_i = ceres::sin(pose_i[2]);
        const T cos_i = ceres::cos(pose_i[2]);
        const T dx = pose_j[0] - pose_i[0];
        const T dy = pose_j[1] - pose_i[1];

        const T z_hat_x     =  cos_i * dx + sin_i * dy;
        const T z_hat_y     = -sin_i * dx + cos_i * dy;
        const T z_hat_theta =  pose_j[2] - pose_i[2];

        const T e_x     = T(edge_.measurement.x)     - z_hat_x;
        const T e_y     = T(edge_.measurement.y)     - z_hat_y;
        const T dtheta  = T(edge_.measurement.theta) - z_hat_theta;
        const T e_theta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));

        Eigen::Matrix<T, 3, 1> e(e_x, e_y, e_theta);
        Eigen::Matrix<T, 3, 1> r = L_.cast<T>() * e;

        residuals[0] = r[0];
        residuals[1] = r[1];
        residuals[2] = r[2];

        return true;
    }

    static ceres::CostFunction* Create(const Edge2D& edge) {
        return new ceres::AutoDiffCostFunction<PoseGraphCostFunctor, 3, 3, 3>(
            new PoseGraphCostFunctor(edge));
    }

private:
    Edge2D edge_;
    Eigen::Matrix3d L_;  // Cholesky factor of Omega: Omega = L * L^T
};

SolverResult solveCeres(PoseGraph& graph, bool verbose) {
    SolverResult result;
    result.solver_name = "Ceres";
    result.initial_cost = graph.totalCost();

    int n = graph.numPoses();
    std::vector<std::array<double, 3>> params(n);
    for (int i = 0; i < n; i++)
        params[i] = {graph.poses[i].x, graph.poses[i].y, graph.poses[i].theta};

    ceres::Problem problem;
    for (int i = 0; i < graph.numEdges(); i++) {
        ceres::CostFunction* cost = PoseGraphCostFunctor::Create(graph.edges[i]);
        problem.AddResidualBlock(cost, nullptr,
                                 params[graph.edges[i].from].data(),
                                 params[graph.edges[i].to].data());
    }
    problem.SetParameterBlockConstant(params[0].data());

    ceres::Solver::Options options;
    options.linear_solver_type        = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations        = 100;
    options.minimizer_progress_to_stdout = verbose;

    ceres::Solver::Summary summary;
    auto t0 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto t1 = std::chrono::steady_clock::now();

    for (int i = 0; i < n; i++) {
        graph.poses[i].x     = params[i][0];
        graph.poses[i].y     = params[i][1];
        graph.poses[i].theta = params[i][2];
    }

    result.final_cost = summary.final_cost;
    result.iterations = summary.iterations.size();
    result.time_ms    = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.converged  = summary.IsSolutionUsable();

    return result;
}

}  // namespace slam_opt
