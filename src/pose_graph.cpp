#include "pose_graph.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace slam_opt {

// ----------------------------------------------------------------
// Angle utility
// ----------------------------------------------------------------
double normaliseAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// ----------------------------------------------------------------
// Pose2D
// ----------------------------------------------------------------
Pose2D Pose2D::compose(const Pose2D& other) const {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Pose2D result;
    result.x     = x + c * other.x - s * other.y;
    result.y     = y + s * other.x + c * other.y;
    result.theta = normaliseAngle(theta + other.theta);
    return result;
}

Pose2D Pose2D::inverse() const {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Pose2D inv;
    inv.x     = -(c * x + s * y);
    inv.y     =  (s * x - c * y);
    inv.theta = normaliseAngle(-theta);
    return inv;
}

Pose2D Pose2D::relativeTo(const Pose2D& to) const {
    return inverse().compose(to);
}

// ----------------------------------------------------------------
// PoseGraph — file I/O
// ----------------------------------------------------------------
bool PoseGraph::loadG2O(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[PoseGraph] Cannot open file: " << path << '\n';
        return false;
    }

    poses.clear();
    edges.clear();

    std::string line;
    int max_id = -1;

    // First pass: discover the number of vertices
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "VERTEX_SE2") {
            int id; double x, y, th;
            ss >> id >> x >> y >> th;
            max_id = std::max(max_id, id);
        }
    }

    if (max_id < 0) {
        std::cerr << "[PoseGraph] No VERTEX_SE2 entries found.\n";
        return false;
    }

    poses.resize(max_id + 1);

    // Second pass: fill vertices and edges
    file.clear();
    file.seekg(0);

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "VERTEX_SE2") {
            int id; double x, y, th;
            ss >> id >> x >> y >> th;
            poses[id] = {x, y, normaliseAngle(th)};

        } else if (tag == "EDGE_SE2") {
            Edge2D e;
            double dx, dy, dth;
            // information matrix (upper-triangular, row-major: i11 i12 i13 i22 i23 i33)
            double i11, i12, i13, i22, i23, i33;
            ss >> e.from >> e.to >> dx >> dy >> dth
               >> i11 >> i12 >> i13 >> i22 >> i23 >> i33;

            e.measurement = {dx, dy, normaliseAngle(dth)};
            e.information <<
                i11, i12, i13,
                i12, i22, i23,
                i13, i23, i33;

            edges.push_back(e);
        }
    }

    initial_poses_ = poses;

    std::cout << "[PoseGraph] Loaded " << poses.size() << " poses and "
              << edges.size() << " edges from " << path << '\n';
    return true;
}

bool PoseGraph::savePoses(const std::string& path,
                          const std::string& header) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[PoseGraph] Cannot write to: " << path << '\n';
        return false;
    }
    if (!header.empty()) f << "# " << header << '\n';
    f << "# x y theta\n";
    for (const auto& p : poses)
        f << p.x << ' ' << p.y << ' ' << p.theta << '\n';
    return true;
}

void PoseGraph::resetPoses() {
    poses = initial_poses_;
}

double PoseGraph::totalCost() const {
    double cost = 0.0;
    for (const auto& e : edges) {
        const Pose2D& pi = poses[e.from];
        const Pose2D& pj = poses[e.to];

        // predicted relative pose
        Pose2D z_hat = pi.relativeTo(pj);

        Eigen::Vector3d err(
            e.measurement.x     - z_hat.x,
            e.measurement.y     - z_hat.y,
            normaliseAngle(e.measurement.theta - z_hat.theta)
        );

        cost += err.transpose() * e.information * err;
    }
    return 0.5 * cost;
}

}  // namespace slam_opt
