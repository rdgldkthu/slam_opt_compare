#include "ceres_solver.hpp"

#include <ceres/ceres.h>
#include <chrono>
#include <iostream>

namespace slam_opt {

// ================================================================
//  TODO  Milestone 2: Implement the Ceres cost functor
// ================================================================

struct PoseGraphCostFunctor {
    explicit PoseGraphCostFunctor(const Edge2D& edge) : edge_(edge) {
        L_ = edge.information.llt().matrixL();
    }

    template <typename T>
    bool operator()(const T* const pose_i,
                    const T* const pose_j,
                    T* residuals) const {
        // Step 1: compute predicted relative pose z_hat from pose_i to pose_j
        const T x_i = pose_i[0];
        const T y_i = pose_i[1];
        const T theta_i = pose_i[2];

        const T x_j = pose_j[0];
        const T y_j = pose_j[1];
        const T theta_j = pose_j[2];

        const T sin_theta_i = ceres::sin(theta_i);
        const T cos_theta_i = ceres::cos(theta_i);

        const T dx = x_j - x_i;
        const T dy = y_j - y_i;

        const T z_hat_x = cos_theta_i * dx + sin_theta_i * dy;
        const T z_hat_y = -sin_theta_i * dx + cos_theta_i * dy;
        const T z_hat_theta = theta_j - theta_i;

        // Step 2: compute raw error e = measurement - z_hat
        //         (don't forget to normalise the angle component!)
        const T e_x = edge_.measurement.x - z_hat_x;
        const T e_y = edge_.measurement.y - z_hat_y;
        const T dtheta = edge_.measurement.theta - z_hat_theta;
        const T e_theta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));

        // Step 3: whiten the error: r = L * e   (L = Cholesky factor of Omega)
        //         Pre-compute L in the constructor.
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
    Eigen::Matrix3d L_;   // sqrt(Omega) = Cholesky factor
};

// ================================================================

SolverResult solveCeres(PoseGraph& graph, bool verbose) {
    SolverResult result;
    result.solver_name = "Ceres";
    result.initial_cost = graph.totalCost();

    // TODO: Build the ceres::Problem
    //   - For each edge, add a residual block using PoseGraphCostFunctor::Create()
    //   - Parameter block for pose i:  &graph.poses[i].x  (3 doubles contiguous)
    //     Hint: Pose2D fields are x, y, theta — are they contiguous in memory?
    //     Consider using a flat double array per pose instead.
    //   - Fix pose 0:  problem.SetParameterBlockConstant(...)
    int n = graph.numPoses();
    std::vector<std::array<double, 3>> params(n);
    for (int i = 0; i < n; i++) {
      params[i] = { graph.poses[i].x, graph.poses[i].y, graph.poses[i].theta};
    }

    ceres::Problem problem;
    for (int i = 0; i < graph.numEdges(); i++) {
        ceres::CostFunction* cost_function = PoseGraphCostFunctor::Create(graph.edges[i]);
        problem.AddResidualBlock(cost_function, nullptr, params[graph.edges[i].from].data(), params[graph.edges[i].to].data());
    }
    problem.SetParameterBlockConstant(params[0].data());

    // TODO: Configure and run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = verbose;

    ceres::Solver::Summary summary;
    auto t0 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto t1 = std::chrono::steady_clock::now();

    for (int i = 0; i < n; i++) {
        graph.poses[i].x = params[i][0];
        graph.poses[i].y = params[i][1];
        graph.poses[i].theta = params[i][2];
    }

    // TODO: Fill in result fields
    result.final_cost  = summary.final_cost;
    result.iterations  = summary.iterations.size();
    result.time_ms     = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.converged   = summary.IsSolutionUsable();

    return result;
}

}  // namespace slam_opt
