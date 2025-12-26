#!/usr/bin/env python3
"""
Generate planning performance plots from metrics data (Section 3.1).

Usage:
  python planning_performance_plots.py --data-dir metrics --output-dir plots

Focuses on:
  - Path length comparison
  - Runtime comparison  
  - Smoothness metrics
  - Success rate
"""

from __future__ import annotations

import argparse
import glob
import os
import re
from typing import Dict, List

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Set publication-quality style
sns.set_style("whitegrid")
plt.rcParams['font.size'] = 11
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['axes.titlesize'] = 13
plt.rcParams['legend.fontsize'] = 10
plt.rcParams['figure.dpi'] = 100

FILENAME_RE = re.compile(
    r"^(?P<planner>.+?)_(?P<controller>.+?)_CBF(?P<cbf>[01])_(?P<estimator>[^_]+)_metrics\.csv$"
)

PLANNER_LABELS = {
    "AStar": "A*",
    "RRTStar": "RRT*",
    "HybridAStar": "Hybrid A*",
}

COLORS = {
    "A*": "#2E86AB",
    "RRT*": "#A23B72", 
    "Hybrid A*": "#F18F01"
}


def normalize_label(planner: str) -> str:
    return PLANNER_LABELS.get(planner, planner)


def parse_file_name(filename: str) -> Dict[str, str] | None:
    base = os.path.basename(filename)
    match = FILENAME_RE.match(base)
    if not match:
        return None
    info = match.groupdict()
    info["cbf"] = "ON" if info["cbf"] == "1" else "OFF"
    return info


def load_metrics(data_dir: str) -> pd.DataFrame:
    records: List[Dict[str, float | str]] = []
    for path in glob.glob(os.path.join(data_dir, "*_metrics.csv")):
        meta = parse_file_name(path)
        if not meta:
            continue
        df = pd.read_csv(path)
        if "Metric" not in df.columns or "Value" not in df.columns:
            continue
        metrics = dict(zip(df["Metric"], df["Value"]))
        row: Dict[str, float | str] = {
            "planner": meta["planner"],
            "controller": meta["controller"],
            "cbf": meta["cbf"],
            "estimator": meta["estimator"],
        }
        row.update(metrics)
        records.append(row)
    return pd.DataFrame.from_records(records)


def filter_data(df: pd.DataFrame, estimator: str | None, cbf: str | None) -> pd.DataFrame:
    filtered = df.copy()
    if estimator:
        filtered = filtered[filtered["estimator"].str.lower() == estimator.lower()]
    if cbf:
        cbf_norm = cbf.upper()
        if cbf_norm in {"1", "ON"}:
            cbf_norm = "ON"
        elif cbf_norm in {"0", "OFF"}:
            cbf_norm = "OFF"
        filtered = filtered[filtered["cbf"].str.upper() == cbf_norm]
    return filtered


def group_stats(df: pd.DataFrame, metric: str) -> pd.DataFrame:
    subset = df[["planner", metric]].copy()
    subset[metric] = pd.to_numeric(subset[metric], errors="coerce")
    grouped = subset.groupby("planner")[metric].agg(["mean", "std", "count", "min", "max"]).reset_index()
    grouped["label"] = grouped["planner"].apply(normalize_label)
    return grouped


def plot_path_length_comparison(df: pd.DataFrame, out_path: str) -> None:
    """Plot 1: Path Length Comparison across planners."""
    if "path_length" not in df.columns:
        print("  ⚠ path_length not found, skipping...")
        return
    
    grouped = group_stats(df, "path_length")
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
    bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                  capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
    
    ax.set_ylabel("Path Length (m)", fontweight='bold', fontsize=13)
    ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=13)
    ax.set_title("Path Length Comparison", fontweight='bold', fontsize=15, pad=15)
    ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
    ax.set_axisbelow(True)
    
    # Add value labels on bars
    for idx, (mean, std, count) in enumerate(zip(grouped["mean"], grouped["std"], grouped["count"])):
        ax.text(idx, mean + std + 1, f"{mean:.1f}m\n±{std:.1f}", 
               ha="center", va="bottom", fontsize=10, fontweight='bold')
        ax.text(idx, -2, f"n={int(count)}", ha="center", va="top", fontsize=9, style='italic')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_runtime_comparison(df: pd.DataFrame, out_path: str) -> None:
    """Plot 2: Mission Completion Time Comparison."""
    # Use simulation_duration instead of planning_time (which is 0 for all)
    time_metric = None
    if "simulation_duration" in df.columns:
        time_metric = "simulation_duration"
        ylabel = "Simulation Duration (seconds)"
        title = "Mission Completion Time Comparison"
    elif "planning_time" in df.columns:
        time_metric = "planning_time"
        ylabel = "Planning Time (seconds)"
        title = "Planning Runtime Comparison"
    
    if not time_metric:
        print("  ⚠ No time metric found, skipping...")
        return
    
    grouped = group_stats(df, time_metric)
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
    bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                  capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
    
    ax.set_ylabel(ylabel, fontweight='bold', fontsize=13)
    ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=13)
    ax.set_title(title, fontweight='bold', fontsize=15, pad=15)
    ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
    ax.set_axisbelow(True)
    
    # Add value labels
    for idx, (mean, std, count) in enumerate(zip(grouped["mean"], grouped["std"], grouped["count"])):
        ax.text(idx, mean + std + 1, f"{mean:.1f}s\n±{std:.1f}", 
               ha="center", va="bottom", fontsize=10, fontweight='bold')
        ax.text(idx, -2, f"n={int(count)}", ha="center", va="top", fontsize=9, style='italic')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_smoothness_metrics(df: pd.DataFrame, out_path: str) -> None:
    """Plot 3: Smoothness Metrics (Curvature and Jerk)."""
    has_curv = "mean_curvature" in df.columns
    has_jerk = "mean_jerk" in df.columns
    
    if not has_curv and not has_jerk:
        print("  ⚠ No smoothness metrics found, skipping...")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle("Path Smoothness Metrics", fontsize=16, fontweight='bold', y=1.00)
    
    # Curvature
    if has_curv:
        grouped = group_stats(df, "mean_curvature")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Mean Curvature", fontweight='bold', fontsize=12)
        ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=12)
        ax.set_title("Mean Curvature", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.01, f"{mean:.3f}\n±{std:.3f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    # Jerk
    if has_jerk:
        grouped = group_stats(df, "mean_jerk")
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Mean Jerk", fontweight='bold', fontsize=12)
        ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=12)
        ax.set_title("Mean Jerk", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.0001, f"{mean:.5f}\n±{std:.5f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_efficiency_and_success(df: pd.DataFrame, out_path: str) -> None:
    """Plot 4: Path Efficiency and Overall Success."""
    has_efficiency = "path_efficiency" in df.columns
    has_overall = "overall_success" in df.columns
    
    if not has_efficiency and not has_overall:
        print("  ⚠ No efficiency/success metrics found, skipping...")
        return
    
    n_plots = sum([has_efficiency, has_overall])
    fig, axes = plt.subplots(1, n_plots, figsize=(8 * n_plots, 6))
    if n_plots == 1:
        axes = [axes]
    
    fig.suptitle("Planning Efficiency & Success Analysis", fontsize=16, fontweight='bold', y=1.00)
    
    plot_idx = 0
    
    # Path Efficiency (main metric showing differences)
    if has_efficiency:
        grouped = group_stats(df, "path_efficiency")
        
        ax = axes[plot_idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Path Efficiency", fontweight='bold', fontsize=12)
        ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=12)
        ax.set_ylim(0, 1.05)
        ax.set_title("Path Efficiency (Optimality)", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        # Add reference line for perfect efficiency
        ax.axhline(y=1.0, color='green', linestyle='--', linewidth=2, alpha=0.5, label='Perfect Efficiency')
        ax.legend(loc='lower right')
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            percentage = mean * 100
            ax.text(idx, mean + std + 0.02, f"{percentage:.1f}%\n±{std:.3f}", 
                   ha="center", va="bottom", fontsize=10, fontweight='bold')
        
        plot_idx += 1
    
    # Overall Success (for completeness)
    if has_overall:
        subset = df[["planner", "overall_success"]].copy()
        subset["overall_success"] = pd.to_numeric(subset["overall_success"], errors="coerce")
        grouped = subset.groupby("planner")["overall_success"].mean().reset_index()
        grouped["label"] = grouped["planner"].apply(normalize_label)
        grouped["rate"] = grouped["overall_success"] * 100
        
        ax = axes[plot_idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        bars = ax.bar(grouped["label"], grouped["rate"], color=colors,
                      edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Success Rate (%)", fontweight='bold', fontsize=12)
        ax.set_xlabel("Planning Algorithm", fontweight='bold', fontsize=12)
        ax.set_ylim(0, 105)
        ax.set_title("Overall Success Rate", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, rate in enumerate(grouped["rate"]):
            ax.text(idx, rate + 2, f"{rate:.1f}%", ha="center", va="bottom", 
                   fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_comprehensive_summary(df: pd.DataFrame, out_path: str) -> None:
    """Plot 5: Comprehensive 2x2 Summary."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Comprehensive Planning Performance Summary", fontsize=18, fontweight='bold', y=0.995)
    
    # 1. Path Length
    if "path_length" in df.columns:
        grouped = group_stats(df, "path_length")
        ax = axes[0, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Path Length (m)", fontweight='bold')
        ax.set_title("Path Length", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.1f}±{std:.1f}", ha="center", va="bottom", fontsize=9)
    
    # 2. Planning Time / Simulation Duration
    time_metric = "simulation_duration" if "simulation_duration" in df.columns else "planning_time"
    if time_metric in df.columns:
        grouped = group_stats(df, time_metric)
        ax = axes[0, 1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ylabel = "Simulation Time (s)" if time_metric == "simulation_duration" else "Planning Time (s)"
        ax.set_ylabel(ylabel, fontweight='bold')
        ax.set_title("Mission Duration" if time_metric == "simulation_duration" else "Runtime", 
                     fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.1f}±{std:.1f}", ha="center", va="bottom", fontsize=9)
    
    # 3. Mean Curvature
    if "mean_curvature" in df.columns:
        grouped = group_stats(df, "mean_curvature")
        ax = axes[1, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Mean Curvature", fontweight='bold')
        ax.set_title("Smoothness (Curvature)", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 4. Path Efficiency (replacing success rate since all are 100%)
    if "path_efficiency" in df.columns:
        grouped = group_stats(df, "path_efficiency")
        ax = axes[1, 1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Path Efficiency", fontweight='bold')
        ax.set_ylim(0.88, 1.02)
        ax.set_title("Path Efficiency", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        ax.axhline(y=1.0, color='green', linestyle='--', linewidth=1.5, alpha=0.4)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            percentage = mean * 100
            ax.text(idx, mean, f"{percentage:.1f}%\n±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def generate_summary_table(df: pd.DataFrame, out_path: str) -> None:
    """Generate a comprehensive summary table."""
    metrics = ["path_length", "simulation_duration", "mean_curvature", "mean_jerk", 
               "path_efficiency", "planning_success", "overall_success", 
               "min_obstacle_distance", "avg_velocity"]
    
    summary_data = []
    for planner in sorted(df["planner"].unique()):
        row = {"Planner": normalize_label(planner)}
        subset = df[df["planner"] == planner]
        
        for metric in metrics:
            if metric in df.columns:
                values = pd.to_numeric(subset[metric], errors="coerce").dropna()
                if len(values) > 0:
                    mean_val = values.mean()
                    std_val = values.std()
                    row[metric] = f"{mean_val:.4f} ± {std_val:.4f}"
                else:
                    row[metric] = "N/A"
        
        summary_data.append(row)
    
    summary_df = pd.DataFrame(summary_data)
    summary_df.to_csv(out_path, index=False)
    print(f"  ✓ Saved: {os.path.basename(out_path)}")
    
    # Also print a formatted table to console
    print("\n" + "="*80)
    print("KEY METRICS SUMMARY")
    print("="*80)
    print(f"{'Planner':<12} {'Path Eff':>10} {'Sim Time':>12} {'Curvature':>12} {'Jerk':>12}")
    print("-"*80)
    for planner in sorted(df["planner"].unique()):
        subset = df[df["planner"] == planner]
        
        # Path efficiency
        eff_str = "N/A"
        if "path_efficiency" in df.columns:
            values = pd.to_numeric(subset["path_efficiency"], errors="coerce").dropna()
            if len(values) > 0:
                eff_str = f"{values.mean()*100:.1f}%"
        
        # Simulation duration
        dur_str = "N/A"
        if "simulation_duration" in df.columns:
            values = pd.to_numeric(subset["simulation_duration"], errors="coerce").dropna()
            if len(values) > 0:
                dur_str = f"{values.mean():.1f}s"
        
        # Curvature
        curv_str = "N/A"
        if "mean_curvature" in df.columns:
            values = pd.to_numeric(subset["mean_curvature"], errors="coerce").dropna()
            if len(values) > 0:
                curv_str = f"{values.mean():.3f}"
        
        # Jerk
        jerk_str = "N/A"
        if "mean_jerk" in df.columns:
            values = pd.to_numeric(subset["mean_jerk"], errors="coerce").dropna()
            if len(values) > 0:
                jerk_str = f"{values.mean():.3f}"
        
        print(f"{normalize_label(planner):<12} {eff_str:>10} {dur_str:>12} {curv_str:>12} {jerk_str:>12}")
    print("="*80)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate planning performance plots from metrics data.")
    parser.add_argument("--data-dir", default="metrics", help="Directory containing *_metrics.csv files.")
    parser.add_argument("--output-dir", default="plots", help="Directory to save plots.")
    parser.add_argument("--estimator", default=None, help="Filter by estimator (e.g., EKF).")
    parser.add_argument("--cbf", default=None, help="Filter by CBF state (ON/OFF/1/0).")
    args = parser.parse_args()

    print("="*80)
    print("PLANNING PERFORMANCE METRICS VISUALIZATION")
    print("="*80)
    
    df = load_metrics(args.data_dir)
    if df.empty:
        print("❌ No metrics files found. Ensure *_metrics.csv files are in the data directory.")
        return

    print(f"✓ Loaded {len(df)} simulation results")
    print(f"  • Planners: {', '.join(sorted(df['planner'].unique()))}")
    print(f"  • Controllers: {', '.join(sorted(df['controller'].unique()))}")
    print(f"  • Estimators: {', '.join(sorted(df['estimator'].unique()))}")
    
    df = filter_data(df, args.estimator, args.cbf)
    if df.empty:
        print("❌ No data after filtering. Check estimator/CBF filters.")
        return

    if args.estimator or args.cbf:
        print(f"\n📊 Filtered to {len(df)} results")
        if args.estimator:
            print(f"  • Estimator: {args.estimator}")
        if args.cbf:
            print(f"  • CBF: {args.cbf}")

    os.makedirs(args.output_dir, exist_ok=True)

    print("\n" + "="*80)
    print("GENERATING PLOTS...")
    print("="*80)

    # Generate the 4 main plots
    print("\n1. Path Length Comparison")
    plot_path_length_comparison(df, os.path.join(args.output_dir, "01_path_length_comparison.png"))

    print("\n2. Runtime Comparison")
    plot_runtime_comparison(df, os.path.join(args.output_dir, "02_runtime_comparison.png"))

    print("\n3. Smoothness Metrics")
    plot_smoothness_metrics(df, os.path.join(args.output_dir, "03_smoothness_metrics.png"))

    print("\n4. Path Efficiency & Success")
    plot_efficiency_and_success(df, os.path.join(args.output_dir, "04_efficiency_and_success.png"))

    print("\n5. Comprehensive Summary (2x2)")
    plot_comprehensive_summary(df, os.path.join(args.output_dir, "05_comprehensive_summary.png"))

    print("\n6. Summary Table (CSV)")
    generate_summary_table(df, os.path.join(args.output_dir, "summary_table.csv"))

    print("\n" + "="*80)
    print("✅ ALL VISUALIZATIONS GENERATED SUCCESSFULLY!")
    print("="*80)
    print(f"\n📁 Output directory: {os.path.abspath(args.output_dir)}")
    print(f"\nGenerated files:")
    for filename in sorted(os.listdir(args.output_dir)):
        if filename.endswith(('.png', '.csv')):
            print(f"  • {filename}")
    print("="*80)


if __name__ == "__main__":
    main()
