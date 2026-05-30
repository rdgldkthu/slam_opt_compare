#pragma once

#include "pose_graph.hpp"

namespace slam_opt {

SolverResult solveCeres(PoseGraph& graph, bool verbose = false);

}  // namespace slam_opt
