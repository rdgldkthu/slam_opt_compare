#include "ceres_solver.hpp"
#include "g2o_solver.hpp"
#include "pose_graph.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static void printUsage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " --dataset <path.g2o> [--verbose] [--no-ceres] [--no-g2o]\n";
}

static void printResult(const slam_opt::SolverResult& r) {
  std::cout << '\n'
            << "┌──────────────────────────────┐\n"
            << "│  Solver: " << std::left << std::setw(18) << r.solver_name
            << "  │\n"
            << "├──────────────────────────────┤\n"
            << "│  Initial cost : " << std::setw(12) << std::fixed
            << std::setprecision(4) << r.initial_cost << " │\n"
            << "│  Final cost   : " << std::setw(12) << r.final_cost << " │\n"
            << "│  Iterations   : " << std::setw(12) << r.iterations << " │\n"
            << "│  Time (ms)    : " << std::setw(12) << r.time_ms << " │\n"
            << "│  Converged    : " << std::setw(12)
            << (r.converged ? "yes" : "no") << " │\n"
            << "└──────────────────────────────┘\n";
}

int main(int argc, char** argv) {
    // ── parse arguments ──────────────────────────────────────────
    std::string dataset_path;
    bool verbose   = false;
    bool run_ceres = true;
    bool run_g2o   = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dataset" && i + 1 < argc)     dataset_path = argv[++i];
        else if (arg == "--verbose")                 verbose  = true;
        else if (arg == "--no-ceres")                run_ceres = false;
        else if (arg == "--no-g2o")                  run_g2o  = false;
        else { printUsage(argv[0]); return 1; }
    }

    if (dataset_path.empty()) { printUsage(argv[0]); return 1; }

    // ── load the pose graph ───────────────────────────────────────
    slam_opt::PoseGraph graph;
    if (!graph.loadG2O(dataset_path)) return 1;

    fs::create_directories("results");

    // ── Ceres ─────────────────────────────────────────────────────
    if (run_ceres) {
        graph.resetPoses();
        auto r = slam_opt::solveCeres(graph, verbose);
        printResult(r);
        graph.savePoses("results/ceres_poses.txt", "Ceres optimised poses");
    }

    // ── g2o ───────────────────────────────────────────────────────
    if (run_g2o) {
        graph.resetPoses();
        auto r = slam_opt::solveG2O(graph, 100, verbose);
        printResult(r);
        graph.savePoses("results/g2o_poses.txt", "g2o optimised poses");
    }

    std::cout << "\nResults written to results/\n"
              << "Visualise with:  python3 scripts/plot_result.py\n";
    return 0;
}
