#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace slam_opt {

// ----------------------------------------------------------------
// 2-D pose: (x, y, theta)   in the world frame
// ----------------------------------------------------------------
struct Pose2D {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;  // radians, normalised to (-pi, pi]

    /// Compose: return the pose obtained by applying `other` in this frame.
    Pose2D compose(const Pose2D& other) const;

    /// Inverse of this pose.
    Pose2D inverse() const;

    /// Relative pose from *this* to `to`  ( = this^{-1} ⊕ to )
    Pose2D relativeTo(const Pose2D& to) const;
};

/// Normalise angle to (-pi, pi]
double normaliseAngle(double angle);

// ----------------------------------------------------------------
// Edge: a relative-pose measurement between pose i and pose j
// ----------------------------------------------------------------
struct Edge2D {
    int from;  // index of source pose
    int to;    // index of target pose

    Pose2D measurement;               ///< z_tilde_ij
    Eigen::Matrix3d information;      ///< Omega_ij = Sigma_ij^{-1}  (3x3)
};

// ----------------------------------------------------------------
// Pose graph: collection of poses + edges
// ----------------------------------------------------------------
class PoseGraph {
public:
    std::vector<Pose2D> poses;   ///< optimisation variables
    std::vector<Edge2D> edges;   ///< constraints

    /// Load a .g2o file (VERTEX_SE2 / EDGE_SE2 format).
    bool loadG2O(const std::string& path);

    /// Save poses to a simple text file (one "x y theta" per line).
    bool savePoses(const std::string& path,
                   const std::string& header = "") const;

    /// Reset poses to their initial (loaded) values.
    void resetPoses();

    /// Compute total weighted squared error given current poses.
    double totalCost() const;

    int numPoses() const { return static_cast<int>(poses.size()); }
    int numEdges() const { return static_cast<int>(edges.size()); }

private:
    std::vector<Pose2D> initial_poses_;  ///< saved on first load
};

// ----------------------------------------------------------------
// Benchmark result returned by each solver
// ----------------------------------------------------------------
struct SolverResult {
    std::string solver_name;
    double      initial_cost   = 0.0;
    double      final_cost     = 0.0;
    int         iterations     = 0;
    double      time_ms        = 0.0;   ///< wall-clock time in ms
    bool        converged      = false;
};

}  // namespace slam_opt
