#!/usr/bin/env python3
"""
Generate example trajectory plots for Section 3.1.

Usage:
  python example_trajectories.py --data-dir . --estimator EKF --cbf ON

This script expects files named like:
  AStar_LQR_CBF1_EKF_trajectory.csv
  RRTStar_MPC_CBF0_Perfect_trajectory.csv
"""

from __future__ import annotations

import argparse
import glob
import os
import re
from typing import Dict, List

import pandas as pd
import matplotlib.pyplot as plt

TRJ_RE = re.compile(
    r"^(?P<planner>.+?)_(?P<controller>.+?)_CBF(?P<cbf>[01])_(?P<estimator>[^_]+)_trajectory\.csv$"
)

PLANNER_LABELS = {
    "AStar": "A*",
    "RRTStar": "RRT*",
    "HybridAStar": "Hybrid A*",
}


def normalize_label(planner: str) -> str:
    return PLANNER_LABELS.get(planner, planner)


def parse_file_name(filename: str) -> Dict[str, str] | None:
    base = os.path.basename(filename)
    match = TRJ_RE.match(base)
    if not match:
        return None
    info = match.groupdict()
    info["cbf"] = "ON" if info["cbf"] == "1" else "OFF"
    return info


def load_metrics(data_dir: str) -> Dict[str, float]:
    metrics_map: Dict[str, float] = {}
    for path in glob.glob(os.path.join(data_dir, "*_metrics.csv")):
        base = os.path.basename(path).replace("_metrics.csv", "")
        df = pd.read_csv(path)
        if "Metric" not in df.columns or "Value" not in df.columns:
            continue
        metric_dict = dict(zip(df["Metric"], df["Value"]))
        if "path_length" in metric_dict:
            try:
                metrics_map[base] = float(metric_dict["path_length"])
            except ValueError:
                continue
    return metrics_map


def select_examples(traj_files: List[str], metrics_map: Dict[str, float]) -> Dict[str, str]:
    planner_best: Dict[str, str] = {}
    planner_score: Dict[str, float] = {}

    for path in traj_files:
        base = os.path.basename(path).replace("_trajectory.csv", "")
        meta = parse_file_name(path)
        if not meta:
            continue
        planner = meta["planner"]
        score = metrics_map.get(base, float("inf"))
        if planner not in planner_score or score < planner_score[planner]:
            planner_score[planner] = score
            planner_best[planner] = path

    return planner_best


def filter_files(files: List[str], estimator: str | None, cbf: str | None, controller: str | None) -> List[str]:
    filtered: List[str] = []
    cbf_norm = None
    if cbf:
        cbf_norm = "ON" if cbf.upper() in {"ON", "1"} else "OFF"

    for path in files:
        meta = parse_file_name(path)
        if not meta:
            continue
        if estimator and meta["estimator"].lower() != estimator.lower():
            continue
        if cbf_norm and meta["cbf"].upper() != cbf_norm:
            continue
        if controller and meta["controller"].lower() != controller.lower():
            continue
        filtered.append(path)

    return filtered


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate example trajectories plot.")
    parser.add_argument("--data-dir", default="metrics", help="Directory containing *_trajectory.csv files.")
    parser.add_argument("--output-dir", default="plots", help="Directory to save plots.")
    parser.add_argument("--estimator", default=None, help="Filter by estimator (e.g., EKF).")
    parser.add_argument("--cbf", default=None, help="Filter by CBF state (ON/OFF/1/0).")
    parser.add_argument("--controller", default=None, help="Filter by controller (e.g., MPC).")
    args = parser.parse_args()

    traj_files = glob.glob(os.path.join(args.data_dir, "*_trajectory.csv"))
    if not traj_files:
        print("No trajectory files found. Ensure *_trajectory.csv files are in the data directory.")
        return

    traj_files = filter_files(traj_files, args.estimator, args.cbf, args.controller)
    if not traj_files:
        print("No trajectories after filtering. Check estimator/CBF/controller filters.")
        return

    metrics_map = load_metrics(args.data_dir)
    examples = select_examples(traj_files, metrics_map)
    if not examples:
        print("No valid trajectories to plot.")
        return

    os.makedirs(args.output_dir, exist_ok=True)

    plt.figure(figsize=(7, 6))
    for planner, path in examples.items():
        df = pd.read_csv(path)
        if "X" not in df.columns or "Y" not in df.columns:
            continue
        label = normalize_label(planner)
        plt.plot(df["X"], df["Y"], linewidth=2, label=label)
        plt.scatter([df["X"].iloc[0]], [df["Y"].iloc[0]], s=30, marker="o")
        plt.scatter([df["X"].iloc[-1]], [df["Y"].iloc[-1]], s=30, marker="x")

    plt.title("Example Trajectories by Planner")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(alpha=0.25)
    plt.legend()
    plt.tight_layout()

    out_path = os.path.join(args.output_dir, "example_trajectories.png")
    plt.savefig(out_path, dpi=200)
    plt.close()

    print(f"Plot saved to: {os.path.abspath(out_path)}")


if __name__ == "__main__":
    main()
