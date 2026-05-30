#pragma once

#include "pose_graph.hpp"

namespace slam_opt {

SolverResult solveG2O(PoseGraph& graph, int max_iter = 100, bool verbose = false);

}  // namespace slam_opt
