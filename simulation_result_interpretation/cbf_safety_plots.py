#!/usr/bin/env python3
"""
Generate CBF Safety Layer Performance plots from metrics data (Section 3.3/3.4).

Usage:
  python cbf_safety_plots.py --data-dir metrics --output-dir plots

Focuses on:
  - Collision avoidance demonstrations
  - Safety margin analysis
  - Control modifications by CBF layer
  - Effect of CBF on tracking accuracy and smoothness
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
    "HybridAStar": "Hybrid A*",
    "RRTStar": "RRT*"
}

CONTROLLER_LABELS = {
    "LQR": "LQR",
    "MPC": "MPC",
    "PID": "PID"
}

CBF_LABELS = {
    "0": "CBF OFF",
    "1": "CBF ON"
}

COLORS = {
    "CBF OFF": "#E63946",  # Red for OFF
    "CBF ON": "#06A77D"     # Green for ON
}


def normalize_label(name: str) -> str:
    """Normalize algorithm/controller names for display."""
    return PLANNER_LABELS.get(name, CONTROLLER_LABELS.get(name, name))


def load_metrics(data_dir: str) -> pd.DataFrame:
    """Load all metrics CSV files from the specified directory."""
    pattern = os.path.join(data_dir, "*_metrics.csv")
    files = glob.glob(pattern)
    
    if not files:
        raise FileNotFoundError(f"No metrics files found in {data_dir}")
    
    all_data = []
    for filepath in files:
        basename = os.path.basename(filepath)
        match = FILENAME_RE.match(basename)
        if not match:
            print(f"Warning: Skipping file with unexpected name: {basename}")
            continue
        
        info = match.groupdict()
        df = pd.read_csv(filepath)
        
        # Transpose if needed (Metric, Value format)
        if "Metric" in df.columns and "Value" in df.columns:
            metrics_dict = dict(zip(df["Metric"], df["Value"]))
            metrics_dict.update(info)
            all_data.append(metrics_dict)
        else:
            # Already in row format
            row = df.iloc[0].to_dict()
            row.update(info)
            all_data.append(row)
    
    return pd.DataFrame(all_data)


def group_stats(df: pd.DataFrame, metric: str, group_by: str = "cbf") -> pd.DataFrame:
    """Compute mean and std dev for a metric grouped by CBF state."""
    subset = df[[group_by, metric]].copy()
    subset[metric] = pd.to_numeric(subset[metric], errors="coerce")
    
    grouped = subset.groupby(group_by)[metric].agg(['mean', 'std']).reset_index()
    grouped['label'] = grouped[group_by].map(CBF_LABELS)
    
    return grouped


def plot_safety_margins(df: pd.DataFrame, out_path: str) -> None:
    """Plot 1: Safety margin comparison (obstacle distances)."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Safety Margin Analysis: CBF Impact on Obstacle Clearance", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    # Min obstacle distance
    if "min_obstacle_distance" in df.columns:
        grouped = group_stats(df, "min_obstacle_distance")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Minimum Obstacle Distance (m)", fontweight='bold')
        ax.set_title("Minimum Safety Clearance", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.05, f"{mean:.3f}±{std:.3f}", 
                   ha="center", va="bottom", fontsize=10, fontweight='bold')
    
    # Mean obstacle distance
    if "mean_obstacle_distance" in df.columns:
        grouped = group_stats(df, "mean_obstacle_distance")
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Mean Obstacle Distance (m)", fontweight='bold')
        ax.set_title("Average Safety Margin", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.05, f"{mean:.3f}±{std:.3f}", 
                   ha="center", va="bottom", fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_cbf_activation_analysis(df: pd.DataFrame, out_path: str) -> None:
    """Plot 2: CBF activation patterns (count, rate, duration)."""
    # Only analyze CBF ON cases
    df_cbf_on = df[df["cbf"] == "1"].copy()
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("CBF Activation Patterns: Safety Layer Intervention Analysis", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    # CBF Activation Count by controller
    if "cbf_activation_count" in df_cbf_on.columns:
        subset = df_cbf_on[["controller", "cbf_activation_count"]].copy()
        subset["cbf_activation_count"] = pd.to_numeric(subset["cbf_activation_count"], errors="coerce")
        grouped = subset.groupby("controller")["cbf_activation_count"].agg(['mean', 'std']).reset_index()
        grouped['label'] = grouped['controller'].apply(normalize_label)
        
        ax = axes[0]
        colors = ["#2E86AB", "#A23B72", "#F18F01"]  # LQR, MPC, PID
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=6, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Activation Count", fontweight='bold')
        ax.set_title("CBF Activation Frequency", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.1f}±{std:.1f}", ha="center", va="bottom", fontsize=9)
    
    # CBF Activation Rate by controller
    if "cbf_activation_rate" in df_cbf_on.columns:
        subset = df_cbf_on[["controller", "cbf_activation_rate"]].copy()
        subset["cbf_activation_rate"] = pd.to_numeric(subset["cbf_activation_rate"], errors="coerce")
        grouped = subset.groupby("controller")["cbf_activation_rate"].agg(['mean', 'std']).reset_index()
        grouped['label'] = grouped['controller'].apply(normalize_label)
        
        ax = axes[1]
        colors = ["#2E86AB", "#A23B72", "#F18F01"]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=6, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Activation Rate (Hz)", fontweight='bold')
        ax.set_title("CBF Intervention Rate", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # CBF Duration by controller
    if "cbf_duration" in df_cbf_on.columns:
        subset = df_cbf_on[["controller", "cbf_duration"]].copy()
        subset["cbf_duration"] = pd.to_numeric(subset["cbf_duration"], errors="coerce")
        grouped = subset.groupby("controller")["cbf_duration"].agg(['mean', 'std']).reset_index()
        grouped['label'] = grouped['controller'].apply(normalize_label)
        
        ax = axes[2]
        colors = ["#2E86AB", "#A23B72", "#F18F01"]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=6, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Total Active Duration (s)", fontweight='bold')
        ax.set_title("CBF Engagement Time", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.1f}±{std:.1f}", ha="center", va="bottom", fontsize=9)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_cbf_impact_on_tracking(df: pd.DataFrame, out_path: str) -> None:
    """Plot 3: Effect of CBF on tracking accuracy."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("CBF Impact on Tracking Performance", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    metrics = ["mean_tracking_error", "max_tracking_error", "rms_tracking_error"]
    titles = ["Mean Tracking Error", "Maximum Tracking Error", "RMS Tracking Error"]
    
    for idx, (metric, title) in enumerate(zip(metrics, titles)):
        if metric not in df.columns:
            continue
        
        grouped = group_stats(df, metric)
        ax = axes[idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel(f"{title} (m)", fontweight='bold')
        ax.set_title(title, fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for i, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(i, mean + std + 0.02, f"{mean:.3f}±{std:.3f}", 
                   ha="center", va="bottom", fontsize=9)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_cbf_impact_on_smoothness(df: pd.DataFrame, out_path: str) -> None:
    """Plot 4: Effect of CBF on control smoothness."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("CBF Impact on Control Smoothness", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    # Mean velocity change
    if "mean_velocity_change" in df.columns:
        grouped = group_stats(df, "mean_velocity_change")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Mean Velocity Change (m/s²)", fontweight='bold')
        ax.set_title("Linear Control Smoothness", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.05, f"{mean:.3f}±{std:.3f}", 
                   ha="center", va="bottom", fontsize=10)
    
    # Mean angular velocity change
    if "mean_omega_change" in df.columns:
        grouped = group_stats(df, "mean_omega_change")
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Mean Angular Change (rad/s²)", fontweight='bold')
        ax.set_title("Angular Control Smoothness", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.1, f"{mean:.3f}±{std:.3f}", 
                   ha="center", va="bottom", fontsize=10)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_safety_violations(df: pd.DataFrame, out_path: str) -> None:
    """Plot 5: Safety violations comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Collision Avoidance: Safety Constraint Satisfaction", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    # Safety violations count
    if "safety_violations" in df.columns:
        grouped = group_stats(df, "safety_violations")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                     capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Safety Violations", fontweight='bold')
        ax.set_title("Safety Constraint Violations", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.1, f"{mean:.2f}±{std:.2f}", 
                   ha="center", va="bottom", fontsize=10, fontweight='bold')
        
        # Highlight zero violations
        if grouped["mean"].min() < 0.1:
            ax.axhline(y=0, color='green', linestyle='--', linewidth=2, alpha=0.5)
            ax.text(0.5, 0.05, "ZERO VIOLATIONS", ha="center", va="bottom",
                   fontsize=9, color='green', fontweight='bold', 
                   transform=ax.get_xaxis_transform())
    
    # Safety maintained percentage
    if "safety_maintained" in df.columns:
        subset = df[["cbf", "safety_maintained"]].copy()
        subset["safety_maintained"] = pd.to_numeric(subset["safety_maintained"], errors="coerce")
        grouped = subset.groupby("cbf")["safety_maintained"].agg(['mean', 'std']).reset_index()
        grouped['label'] = grouped['cbf'].map(CBF_LABELS)
        grouped['rate'] = grouped['mean'] * 100
        grouped['std_pct'] = grouped['std'] * 100
        
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["rate"], yerr=grouped["std_pct"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Safety Maintained (%)", fontweight='bold')
        ax.set_ylim(0, 105)
        ax.set_title("Safety Constraint Satisfaction Rate", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (rate, std) in enumerate(zip(grouped["rate"], grouped["std_pct"])):
            ax.text(idx, rate + 2, f"{rate:.1f}%", ha="center", va="bottom", 
                   fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_comprehensive_summary(df: pd.DataFrame, out_path: str) -> None:
    """Plot 6: Comprehensive 2x2 CBF Summary."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Comprehensive CBF Safety Layer Performance", 
                 fontsize=18, fontweight='bold', y=0.995)
    
    # 1. Minimum Obstacle Distance
    if "min_obstacle_distance" in df.columns:
        grouped = group_stats(df, "min_obstacle_distance")
        ax = axes[0, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Min Obstacle Distance (m)", fontweight='bold')
        ax.set_title("Safety Margin", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 2. Mean Tracking Error (CBF Impact)
    if "mean_tracking_error" in df.columns:
        grouped = group_stats(df, "mean_tracking_error")
        ax = axes[0, 1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Mean Tracking Error (m)", fontweight='bold')
        ax.set_title("Tracking Accuracy Trade-off", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 3. Control Smoothness Impact
    if "mean_velocity_change" in df.columns:
        grouped = group_stats(df, "mean_velocity_change")
        ax = axes[1, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Mean Velocity Change (m/s²)", fontweight='bold')
        ax.set_title("Control Smoothness Trade-off", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 4. Safety Violations
    if "safety_violations" in df.columns:
        grouped = group_stats(df, "safety_violations")
        ax = axes[1, 1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=8, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        ax.set_ylabel("Safety Violations", fontweight='bold')
        ax.set_title("Collision Avoidance Success", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.2f}±{std:.2f}", ha="center", va="bottom", fontsize=9)
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def plot_cbf_controller_heatmap(df: pd.DataFrame, out_path: str) -> None:
    """Plot 7: Heatmap showing CBF impact across controllers."""
    df_pivot = df.copy()
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle("CBF Safety Impact Across Controllers", 
                 fontsize=16, fontweight='bold', y=1.00)
    
    # Tracking error heatmap
    if "mean_tracking_error" in df.columns:
        subset = df[["controller", "cbf", "mean_tracking_error"]].copy()
        subset["mean_tracking_error"] = pd.to_numeric(subset["mean_tracking_error"], errors="coerce")
        subset["cbf_label"] = subset["cbf"].map(CBF_LABELS)
        
        pivot = subset.pivot_table(values="mean_tracking_error", 
                                   index="controller", columns="cbf_label", aggfunc="mean")
        
        ax = axes[0]
        sns.heatmap(pivot, annot=True, fmt=".3f", cmap="RdYlGn_r", 
                   cbar_kws={'label': 'Tracking Error (m)'}, ax=ax,
                   linewidths=0.5, linecolor='gray')
        ax.set_title("Tracking Error: CBF Impact", fontweight='bold', pad=10)
        ax.set_xlabel("CBF State", fontweight='bold')
        ax.set_ylabel("Controller", fontweight='bold')
    
    # Safety margin heatmap
    if "min_obstacle_distance" in df.columns:
        subset = df[["controller", "cbf", "min_obstacle_distance"]].copy()
        subset["min_obstacle_distance"] = pd.to_numeric(subset["min_obstacle_distance"], errors="coerce")
        subset["cbf_label"] = subset["cbf"].map(CBF_LABELS)
        
        pivot = subset.pivot_table(values="min_obstacle_distance", 
                                   index="controller", columns="cbf_label", aggfunc="mean")
        
        ax = axes[1]
        sns.heatmap(pivot, annot=True, fmt=".3f", cmap="RdYlGn", 
                   cbar_kws={'label': 'Min Distance (m)'}, ax=ax,
                   linewidths=0.5, linecolor='gray')
        ax.set_title("Safety Margin: CBF Impact", fontweight='bold', pad=10)
        ax.set_xlabel("CBF State", fontweight='bold')
        ax.set_ylabel("Controller", fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def generate_summary_table(df: pd.DataFrame, out_path: str) -> None:
    """Generate a comprehensive CBF summary table."""
    metrics = ["min_obstacle_distance", "mean_obstacle_distance", 
               "safety_violations", "safety_maintained",
               "mean_tracking_error", "mean_velocity_change",
               "cbf_activation_count", "cbf_activation_rate", "cbf_duration"]
    
    summary_data = []
    for cbf_state in sorted(df["cbf"].unique()):
        row = {"CBF_State": CBF_LABELS.get(cbf_state, cbf_state)}
        subset = df[df["cbf"] == cbf_state]
        
        for metric in metrics:
            if metric in df.columns:
                values = pd.to_numeric(subset[metric], errors="coerce").dropna()
                if len(values) > 0:
                    mean_val = values.mean()
                    std_val = values.std()
                    row[metric] = f"{mean_val:.4f} ± {std_val:.4f}"
                else:
                    row[metric] = "N/A"
            else:
                row[metric] = "N/A"
        
        summary_data.append(row)
    
    summary_df = pd.DataFrame(summary_data)
    summary_df.to_csv(out_path, index=False)
    print(f"  [OK] Saved: {os.path.basename(out_path)}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate CBF Safety Layer Performance plots"
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default="metrics",
        help="Directory containing metrics CSV files"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="plots",
        help="Directory to save output plots"
    )
    args = parser.parse_args()
    
    # Load data
    print("=" * 80)
    print("CBF SAFETY LAYER PERFORMANCE VISUALIZATION")
    print("=" * 80)
    df = load_metrics(args.data_dir)
    print(f"[OK]: Loaded {len(df)} simulation results")
    
    # Print data summary
    print(f"  - Planners: {', '.join(sorted(df['planner'].unique()))}")
    print(f"  - Controllers: {', '.join(sorted(df['controller'].unique()))}")
    print(f"  - CBF States: {', '.join([CBF_LABELS[c] for c in sorted(df['cbf'].unique())])}")
    print(f"  - Estimators: {', '.join(sorted(df['estimator'].unique()))}")
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print("\n" + "=" * 80)
    print("GENERATING PLOTS...")
    print("=" * 80 + "\n")
    
    # Generate plots
    print("1. Safety Margin Analysis")
    plot_safety_margins(df, os.path.join(args.output_dir, "cbf_01_safety_margins.png"))
    
    print("\n2. CBF Activation Patterns")
    plot_cbf_activation_analysis(df, os.path.join(args.output_dir, "cbf_02_activation_patterns.png"))
    
    print("\n3. CBF Impact on Tracking")
    plot_cbf_impact_on_tracking(df, os.path.join(args.output_dir, "cbf_03_tracking_impact.png"))
    
    print("\n4. CBF Impact on Smoothness")
    plot_cbf_impact_on_smoothness(df, os.path.join(args.output_dir, "cbf_04_smoothness_impact.png"))
    
    print("\n5. Safety Violations Analysis")
    plot_safety_violations(df, os.path.join(args.output_dir, "cbf_05_safety_violations.png"))
    
    print("\n6. Comprehensive Summary (2x2)")
    plot_comprehensive_summary(df, os.path.join(args.output_dir, "cbf_06_comprehensive_summary.png"))
    
    print("\n7. CBF-Controller Heatmaps")
    plot_cbf_controller_heatmap(df, os.path.join(args.output_dir, "cbf_07_controller_heatmaps.png"))
    
    print("\n8. Summary Table")
    generate_summary_table(df, os.path.join(args.output_dir, "cbf_summary_table.csv"))
    
    # Print final statistics
    print("\n" + "=" * 80)
    print("CBF SAFETY ANALYSIS SUMMARY")
    print("=" * 80)
    
    for cbf_state in sorted(df["cbf"].unique()):
        subset = df[df["cbf"] == cbf_state]
        cbf_label = CBF_LABELS[cbf_state]
        print(f"\n{cbf_label}:")
        
        if "min_obstacle_distance" in df.columns:
            values = pd.to_numeric(subset["min_obstacle_distance"], errors="coerce").dropna()
            if len(values) > 0:
                print(f"  - Min Obstacle Distance: {values.mean():.3f} ± {values.std():.3f} m")
        
        if "safety_violations" in df.columns:
            values = pd.to_numeric(subset["safety_violations"], errors="coerce").dropna()
            if len(values) > 0:
                print(f"  - Safety Violations: {values.mean():.2f} ± {values.std():.2f}")
        
        if "mean_tracking_error" in df.columns:
            values = pd.to_numeric(subset["mean_tracking_error"], errors="coerce").dropna()
            if len(values) > 0:
                print(f"  - Mean Tracking Error: {values.mean():.3f} ± {values.std():.3f} m")
        
        if cbf_state == "1" and "cbf_activation_count" in df.columns:
            values = pd.to_numeric(subset["cbf_activation_count"], errors="coerce").dropna()
            if len(values) > 0:
                print(f"  - CBF Activations: {values.mean():.1f} ± {values.std():.1f}")
    
    print("\n" + "=" * 80)
    print("COMPLETE!")
    print("=" * 80)


if __name__ == "__main__":
    main()
