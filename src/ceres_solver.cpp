#include "ceres_solver.hpp"

#include <ceres/ceres.h>
#include <chrono>
#include <iostream>

namespace slam_opt {

// ================================================================
//  TODO  Milestone 2: Implement the Ceres cost functor
// ================================================================
//
// struct PoseGraphCostFunctor {
//     explicit PoseGraphCostFunctor(const Edge2D& edge) : edge_(edge) {}
//
//     template <typename T>
//     bool operator()(const T* const pose_i,
//                     const T* const pose_j,
//                     T* residuals) const {
//         // Step 1: compute predicted relative pose z_hat from pose_i to pose_j
//         // Step 2: compute raw error e = measurement - z_hat
//         //         (don't forget to normalise the angle component!)
//         // Step 3: whiten the error: r = L * e   (L = Cholesky factor of Omega)
//         //         Pre-compute L in the constructor.
//         return true;
//     }
//
//     static ceres::CostFunction* Create(const Edge2D& edge) {
//         return new ceres::AutoDiffCostFunction<PoseGraphCostFunctor, 3, 3, 3>(
//             new PoseGraphCostFunctor(edge));
//     }
//
// private:
//     Edge2D edge_;
//     Eigen::Matrix3d L_;   // sqrt(Omega) = Cholesky factor
// };
//
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

    // TODO: Configure and run the solver
    //   ceres::Solver::Options options;
    //   options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //   options.max_num_iterations = 100;
    //   options.minimizer_progress_to_stdout = verbose;
    //
    //   ceres::Solver::Summary summary;
    //   auto t0 = std::chrono::steady_clock::now();
    //   ceres::Solve(options, &problem, &summary);
    //   auto t1 = std::chrono::steady_clock::now();

    // TODO: Fill in result fields
    //   result.final_cost  = ...
    //   result.iterations  = ...
    //   result.time_ms     = ...
    //   result.converged   = ...

    std::cout << "[Ceres] NOT YET IMPLEMENTED\n";
    return result;
}

}  // namespace slam_opt
