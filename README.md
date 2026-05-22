# slam_opt_compare

> **2D Pose Graph Optimisation** — Ceres Solver vs g2o, side by side.

A learning-oriented C++ project that solves the canonical SLAM pose-graph
problem using two industry-standard non-linear least-squares backends, then
compares their convergence speed, final cost, and trajectory accuracy.

## The Problem

Given a robot trajectory with noisy odometry edges and loop-closure
constraints, find the set of 2D poses $(x, y, \theta)$ that minimises:

$$
\mathcal{X}^* = \arg\min \sum_{(i,j)} \mathbf{e}_{ij}^T \; \Omega_{ij} \; \mathbf{e}_{ij}
$$

Dataset used: **Manhattan 3500** (3 500 nodes, 5 598 edges).

## Quick Start

```bash
# Install dependencies (Ubuntu 22.04)
sudo apt install libceres-dev libg2o-dev libeigen3-dev python3-matplotlib

# Download dataset
bash scripts/download_data.sh

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..

# Run
./build/slam_opt_compare --dataset data/manhattan3500.g2o

# Visualise
python3 scripts/plot_result.py
```

## Project Structure

```
├── include/
│   ├── pose_graph.hpp      # Pose2D, Edge2D, PoseGraph, SolverResult
│   ├── ceres_solver.hpp    # Ceres solver interface
│   └── g2o_solver.hpp      # g2o solver interface
├── src/
│   ├── main.cpp            # CLI entry point
│   ├── pose_graph.cpp      # .g2o loader, cost computation
│   ├── ceres_solver.cpp    # ← implement this
│   └── g2o_solver.cpp      # ← implement this
├── data/                   # datasets (downloaded separately)
├── scripts/
│   ├── download_data.sh
│   └── plot_result.py
└── results/                # solver output files
```

## Dependencies

| Library | Purpose |
|---------|---------|
| [Eigen3](https://eigen.tuxfamily.org/) | Matrix math |
| [Ceres Solver](http://ceres-solver.org/) | Non-linear LS backend 1 |
| [g2o](https://github.com/RainerKuemmerle/g2o) | Non-linear LS backend 2 |

## Learning Goals

- Understand the SE(2) pose composition and its role in the residual
- Implement a Ceres `AutoDiffCostFunction` from scratch
- Implement custom g2o `BaseVertex` and `BaseBinaryEdge` types
- Compare automatic differentiation vs numerical/analytic Jacobians
- Analyse convergence behaviour on a real-world benchmark dataset

## License

MIT
