#!/usr/bin/env python3
"""
Visualise pose graph optimisation results.

Usage:
    python3 scripts/plot_result.py [--dataset data/manhattan3500.g2o]
                                   [--ceres results/ceres_poses.txt]
                                   [--g2o   results/g2o_poses.txt]
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path


def load_g2o(path):
    """Return (poses, edges) from a .g2o file."""
    poses, edges = {}, []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "VERTEX_SE2":
                idx, x, y, th = int(parts[1]), *map(float, parts[2:5])
                poses[idx] = (x, y, th)
            elif parts[0] == "EDGE_SE2":
                fi, ti = int(parts[1]), int(parts[2])
                edges.append((fi, ti))
    ordered = [poses[i] for i in sorted(poses)]
    return np.array(ordered), edges


def load_poses(path):
    """Load poses from a simple 'x y theta' text file."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(list(map(float, line.split())))
    return np.array(rows)


def draw_trajectory(ax, poses, edges, color, label, alpha=0.8):
    xy = poses[:, :2]
    # edges
    for fi, ti in edges:
        if fi < len(poses) and ti < len(poses):
            ax.plot([poses[fi, 0], poses[ti, 0]],
                    [poses[fi, 1], poses[ti, 1]],
                    color=color, lw=0.4, alpha=0.3)
    # trajectory
    ax.plot(xy[:, 0], xy[:, 1], '-', color=color, lw=1.0,
            alpha=alpha, label=label)
    ax.plot(xy[0, 0], xy[0, 1], 'o', color=color, ms=5)  # start


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data/manhattan3500.g2o")
    parser.add_argument("--ceres",   default="results/ceres_poses.txt")
    parser.add_argument("--g2o",     default="results/g2o_poses.txt")
    args = parser.parse_args()

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("2D Pose Graph Optimisation: Ceres vs g2o", fontsize=14)

    # --- Before optimisation (raw odometry from dataset) ---
    if Path(args.dataset).exists():
        raw_poses, edges = load_g2o(args.dataset)
        draw_trajectory(axes[0], raw_poses, edges, "#888888", "Raw odometry")
        axes[0].set_title("Before optimisation (raw odometry)")
    else:
        axes[0].text(0.5, 0.5, f"Dataset not found:\n{args.dataset}",
                     ha="center", va="center", transform=axes[0].transAxes)

    # --- After: Ceres ---
    if Path(args.ceres).exists():
        ceres_poses = load_poses(args.ceres)
        if Path(args.dataset).exists():
            draw_trajectory(axes[1], ceres_poses, edges, "#e74c3c", "Ceres")
        else:
            axes[1].plot(ceres_poses[:, 0], ceres_poses[:, 1],
                         '-', color="#e74c3c", label="Ceres")
        axes[1].set_title("After: Ceres")
    else:
        axes[1].text(0.5, 0.5, "No Ceres results yet.\nRun the solver first.",
                     ha="center", va="center", transform=axes[1].transAxes)

    # --- After: g2o ---
    if Path(args.g2o).exists():
        g2o_poses = load_poses(args.g2o)
        if Path(args.dataset).exists():
            draw_trajectory(axes[2], g2o_poses, edges, "#2ecc71", "g2o")
        else:
            axes[2].plot(g2o_poses[:, 0], g2o_poses[:, 1],
                         '-', color="#2ecc71", label="g2o")
        axes[2].set_title("After: g2o")
    else:
        axes[2].text(0.5, 0.5, "No g2o results yet.\nRun the solver first.",
                     ha="center", va="center", transform=axes[2].transAxes)

    # --- Overlay comparison ---
    if Path(args.ceres).exists() and Path(args.g2o).exists():
        fig2, ax2 = plt.subplots(figsize=(8, 8))
        ax2.set_title("Overlay comparison (Ceres vs g2o)")
        if Path(args.dataset).exists():
            _, edges = load_g2o(args.dataset)
        else:
            edges = []
        draw_trajectory(ax2, load_poses(args.ceres), edges, "#e74c3c", "Ceres", alpha=0.7)
        draw_trajectory(ax2, load_poses(args.g2o),   edges, "#2ecc71", "g2o",   alpha=0.7)
        ax2.legend()
        ax2.set_aspect("equal")
        ax2.grid(True, alpha=0.3)

    for ax in axes:
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    plt.tight_layout()
    out = Path("results/comparison.png")
    out.parent.mkdir(exist_ok=True)
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved to {out}")
    plt.show()


if __name__ == "__main__":
    main()
