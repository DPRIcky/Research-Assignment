#!/usr/bin/env python3
"""
Generate controller performance plots from metrics data (Section 3.2).

Usage:
  python controller_performance_plots.py --data-dir metrics --output-dir plots

Focuses on:
  - Tracking error
  - Control input smoothness
  - Response to disturbances (CBF activation)
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

CONTROLLER_LABELS = {
    "LQR": "LQR",
    "MPC": "MPC",
    "PID": "PID"
}

COLORS = {
    "LQR": "#2E86AB",
    "MPC": "#A23B72", 
    "PID": "#F18F01"
}


def normalize_label(controller: str) -> str:
    return CONTROLLER_LABELS.get(controller, controller)


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


def filter_data(df: pd.DataFrame, planner: str | None, estimator: str | None, cbf: str | None) -> pd.DataFrame:
    filtered = df.copy()
    if planner:
        filtered = filtered[filtered["planner"].str.lower() == planner.lower()]
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


def group_stats(df: pd.DataFrame, metric: str, groupby: str = "controller") -> pd.DataFrame:
    subset = df[[groupby, metric]].copy()
    subset[metric] = pd.to_numeric(subset[metric], errors="coerce")
    grouped = subset.groupby(groupby)[metric].agg(["mean", "std", "count", "min", "max"]).reset_index()
    if groupby == "controller":
        grouped["label"] = grouped["controller"].apply(normalize_label)
    else:
        grouped["label"] = grouped[groupby]
    return grouped


def plot_tracking_error(df: pd.DataFrame, out_path: str) -> None:
    """Plot 1: Tracking Error Comparison (Mean, Max, RMS)."""
    metrics = ["mean_tracking_error", "max_tracking_error", "rms_tracking_error"]
    available = [m for m in metrics if m in df.columns]
    
    if not available:
        print("  ⚠ No tracking error metrics found, skipping...")
        return
    
    fig, axes = plt.subplots(1, len(available), figsize=(6 * len(available), 6))
    if len(available) == 1:
        axes = [axes]
    
    fig.suptitle("Tracking Error Analysis", fontsize=16, fontweight='bold', y=1.00)
    
    titles = {
        "mean_tracking_error": "Mean Tracking Error",
        "max_tracking_error": "Maximum Tracking Error",
        "rms_tracking_error": "RMS Tracking Error"
    }
    
    for idx, metric in enumerate(available):
        grouped = group_stats(df, metric)
        ax = axes[idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Error (m)", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title(titles[metric], fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for i, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(i, mean + std + 0.02, f"{mean:.3f}\n±{std:.3f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_control_smoothness(df: pd.DataFrame, out_path: str) -> None:
    """Plot 2: Control Input Smoothness."""
    has_vel = "mean_velocity_change" in df.columns
    has_omega = "mean_omega_change" in df.columns
    
    if not has_vel and not has_omega:
        print("  ⚠ No control smoothness metrics found, skipping...")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle("Control Input Smoothness", fontsize=16, fontweight='bold', y=1.00)
    
    # Linear velocity changes
    if has_vel:
        grouped = group_stats(df, "mean_velocity_change")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Mean Velocity Change (m/s²)", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("Linear Velocity Smoothness", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.01, f"{mean:.3f}\n±{std:.3f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    # Angular velocity changes
    if has_omega:
        grouped = group_stats(df, "mean_omega_change")
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Mean Angular Velocity Change (rad/s²)", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("Angular Velocity Smoothness", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.1, f"{mean:.3f}\n±{std:.3f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_control_effort(df: pd.DataFrame, out_path: str) -> None:
    """Plot 3: Control Effort."""
    has_vel = "total_velocity_effort" in df.columns
    has_omega = "total_omega_effort" in df.columns
    
    if not has_vel and not has_omega:
        print("  ⚠ No control effort metrics found, skipping...")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.suptitle("Control Effort Analysis", fontsize=16, fontweight='bold', y=1.00)
    
    # Total velocity effort
    if has_vel:
        grouped = group_stats(df, "total_velocity_effort")
        ax = axes[0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Total Velocity Effort", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("Linear Control Effort", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 1, f"{mean:.1f}\n±{std:.1f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    # Total omega effort
    if has_omega:
        grouped = group_stats(df, "total_omega_effort")
        ax = axes[1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("Total Angular Effort", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("Angular Control Effort", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.5, f"{mean:.2f}\n±{std:.2f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_cbf_response(df: pd.DataFrame, out_path: str) -> None:
    """Plot 4: CBF Activation (Response to Disturbances/Constraints)."""
    has_count = "cbf_activation_count" in df.columns
    has_rate = "cbf_activation_rate" in df.columns
    has_duration = "cbf_duration" in df.columns
    
    if not has_count and not has_rate:
        print("  ⚠ No CBF metrics found, skipping...")
        return
    
    n_plots = sum([has_count, has_duration])
    if n_plots == 0:
        n_plots = 1
    
    fig, axes = plt.subplots(1, n_plots, figsize=(8 * n_plots, 6))
    if n_plots == 1:
        axes = [axes]
    
    fig.suptitle("CBF Activation Analysis (Safety Response)", fontsize=16, fontweight='bold', y=1.00)
    
    plot_idx = 0
    
    # CBF Activation Count
    if has_count:
        grouped = group_stats(df, "cbf_activation_count")
        ax = axes[plot_idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("CBF Activation Count", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("CBF Activation Frequency", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 2, f"{mean:.1f}\n±{std:.1f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
        
        plot_idx += 1
    
    # CBF Duration
    if has_duration:
        grouped = group_stats(df, "cbf_duration")
        ax = axes[plot_idx]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        
        bars = ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
                      capsize=6, color=colors, edgecolor='black', linewidth=1.5, alpha=0.85)
        
        ax.set_ylabel("CBF Duration (s)", fontweight='bold', fontsize=12)
        ax.set_xlabel("Controller", fontweight='bold', fontsize=12)
        ax.set_title("Total CBF Active Time", fontweight='bold', fontsize=14, pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--', linewidth=0.8)
        ax.set_axisbelow(True)
        
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean + std + 0.3, f"{mean:.2f}s\n±{std:.2f}", 
                   ha="center", va="bottom", fontsize=9, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_comprehensive_summary(df: pd.DataFrame, out_path: str) -> pd.DataFrame:
    """Plot 5: Comprehensive 2x2 Summary."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Comprehensive Controller Performance Summary", fontsize=18, fontweight='bold', y=0.995)
    
    # 1. Mean Tracking Error
    if "mean_tracking_error" in df.columns:
        grouped = group_stats(df, "mean_tracking_error")
        ax = axes[0, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Mean Tracking Error (m)", fontweight='bold')
        ax.set_title("Tracking Accuracy", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 2. Control Smoothness (mean velocity change)
    if "mean_velocity_change" in df.columns:
        grouped = group_stats(df, "mean_velocity_change")
        ax = axes[0, 1]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("Mean Velocity Change (m/s²)", fontweight='bold')
        ax.set_title("Control Smoothness", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.3f}±{std:.3f}", ha="center", va="bottom", fontsize=9)
    
    # 3. CBF Activation Count
    if "cbf_activation_count" in df.columns:
        grouped = group_stats(df, "cbf_activation_count")
        ax = axes[1, 0]
        colors = [COLORS.get(label, "#555555") for label in grouped["label"]]
        ax.bar(grouped["label"], grouped["mean"], yerr=grouped["std"],
               capsize=5, color=colors, edgecolor='black', linewidth=1.2, alpha=0.85)
        ax.set_ylabel("CBF Activations", fontweight='bold')
        ax.set_title("Safety Response Frequency", fontweight='bold', pad=10)
        ax.grid(axis="y", alpha=0.3, linestyle='--')
        ax.set_axisbelow(True)
        for idx, (mean, std) in enumerate(zip(grouped["mean"], grouped["std"])):
            ax.text(idx, mean, f"{mean:.1f}±{std:.1f}", ha="center", va="bottom", fontsize=9)
    
    # 4. Tracking Error Heatmap (Controller-Estimator)
    if "mean_tracking_error" in df.columns:
        ax = axes[1, 1]
        subset = df[["controller", "estimator", "mean_tracking_error"]].copy()
        subset["mean_tracking_error"] = pd.to_numeric(subset["mean_tracking_error"], errors="coerce")
        
        pivot = subset.pivot_table(values="mean_tracking_error", index="controller", 
                                   columns="estimator", aggfunc="mean")
        
        sns.heatmap(pivot, annot=True, fmt=".3f", cmap="RdYlGn_r", 
                   cbar_kws={'label': 'Tracking Error (m)'}, ax=ax,
                   linewidths=0.5, linecolor='gray')
        ax.set_title("Tracking Error: Controller-Estimator", fontweight='bold', pad=10)
        ax.set_xlabel("Estimator", fontweight='bold')
        ax.set_ylabel("Controller", fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Saved: {os.path.basename(out_path)}")


def plot_controller_estimator_heatmap(df: pd.DataFrame, metric: str, out_path: str) -> None:
    """Create a heatmap showing metric performance across controller-estimator combinations."""
    if metric not in df.columns:
        return
    
    subset = df[["controller", "estimator", metric]].copy()
    subset[metric] = pd.to_numeric(subset[metric], errors="coerce")
    
    pivot = subset.pivot_table(values=metric, index="controller", columns="estimator", aggfunc="mean")
    
    plt.figure(figsize=(10, 6))
    sns.heatmap(pivot, annot=True, fmt=".3f", cmap="RdYlGn_r", cbar_kws={'label': metric},
                linewidths=0.5, linecolor='gray')
    plt.title(f"{metric.replace('_', ' ').title()} by Controller-Estimator", fontweight='bold', pad=15)
    plt.xlabel("Estimator", fontweight='bold')
    plt.ylabel("Controller", fontweight='bold')
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()


def generate_summary_table(df: pd.DataFrame, out_path: str) -> None:
    """Generate a comprehensive summary table."""
    metrics = ["mean_tracking_error", "rms_tracking_error", "mean_velocity_change", 
               "mean_omega_change", "total_velocity_effort", "total_omega_effort",
               "cbf_activation_count", "cbf_duration", "execution_success"]
    
    summary_data = []
    for controller in sorted(df["controller"].unique()):
        row = {"Controller": normalize_label(controller)}
        subset = df[df["controller"] == controller]
        
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
    print(f"  [OK] Saved: {os.path.basename(out_path)}")
    
    # Also print a formatted table to console
    print("\n" + "="*80)
    print("CONTROLLER PERFORMANCE SUMMARY")
    print("="*80)
    print(f"{'Controller':<12} {'Track Err':>11} {'Smoothness':>12} {'CBF Act':>10} {'Success':>10}")
    print("-"*80)
    for controller in sorted(df["controller"].unique()):
        subset = df[df["controller"] == controller]
        
        # Tracking error
        track_str = "N/A"
        if "mean_tracking_error" in df.columns:
            values = pd.to_numeric(subset["mean_tracking_error"], errors="coerce").dropna()
            if len(values) > 0:
                track_str = f"{values.mean():.3f}m"
        
        # Smoothness
        smooth_str = "N/A"
        if "mean_velocity_change" in df.columns:
            values = pd.to_numeric(subset["mean_velocity_change"], errors="coerce").dropna()
            if len(values) > 0:
                smooth_str = f"{values.mean():.3f}"
        
        # CBF Activation
        cbf_str = "N/A"
        if "cbf_activation_count" in df.columns:
            values = pd.to_numeric(subset["cbf_activation_count"], errors="coerce").dropna()
            if len(values) > 0:
                cbf_str = f"{values.mean():.1f}"
        
        # Success
        success_str = "N/A"
        if "execution_success" in df.columns:
            values = pd.to_numeric(subset["execution_success"], errors="coerce").dropna()
            if len(values) > 0:
                success_str = f"{values.mean()*100:.1f}%"
        
        print(f"{normalize_label(controller):<12} {track_str:>11} {smooth_str:>12} {cbf_str:>10} {success_str:>10}")
    print("="*80)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate controller performance plots from metrics data.")
    parser.add_argument("--data-dir", default="metrics", help="Directory containing *_metrics.csv files.")
    parser.add_argument("--output-dir", default="plots", help="Directory to save plots.")
    parser.add_argument("--planner", default=None, help="Filter by planner (e.g., AStar).")
    parser.add_argument("--estimator", default=None, help="Filter by estimator (e.g., EKF).")
    parser.add_argument("--cbf", default=None, help="Filter by CBF state (ON/OFF/1/0).")
    args = parser.parse_args()

    print("="*80)
    print("CONTROLLER PERFORMANCE METRICS VISUALIZATION")
    print("="*80)
    
    df = load_metrics(args.data_dir)
    if df.empty:
        print("ERROR: No metrics files found. Ensure *_metrics.csv files are in the data directory.")
        return

    print(f"OK: Loaded {len(df)} simulation results")
    print(f"  - Planners: {', '.join(sorted(df['planner'].unique()))}")
    print(f"  - Controllers: {', '.join(sorted(df['controller'].unique()))}")
    print(f"  - Estimators: {', '.join(sorted(df['estimator'].unique()))}")
    
    df_original = df.copy()
    df = filter_data(df, args.planner, args.estimator, args.cbf)
    if df.empty:
        print("ERROR: No data after filtering. Check planner/estimator/CBF filters.")
        return

    if args.planner or args.estimator or args.cbf:
        print(f"\nFiltered to {len(df)} results")
        if args.planner:
            print(f"  - Planner: {args.planner}")
        if args.estimator:
            print(f"  - Estimator: {args.estimator}")
        if args.cbf:
            print(f"  - CBF: {args.cbf}")

    os.makedirs(args.output_dir, exist_ok=True)

    print("\n" + "="*80)
    print("GENERATING PLOTS...")
    print("="*80)

    # Generate controller performance plots
    print("\n1. Tracking Error Analysis")
    plot_tracking_error(df, os.path.join(args.output_dir, "ctrl_01_tracking_error.png"))

    print("\n2. Control Input Smoothness")
    plot_control_smoothness(df, os.path.join(args.output_dir, "ctrl_02_control_smoothness.png"))

    print("\n3. Control Effort")
    plot_control_effort(df, os.path.join(args.output_dir, "ctrl_03_control_effort.png"))

    print("\n4. CBF Response (Safety)")
    plot_cbf_response(df, os.path.join(args.output_dir, "ctrl_04_cbf_response.png"))

    print("\n5. Comprehensive Summary (2x2)")
    plot_comprehensive_summary(df, os.path.join(args.output_dir, "ctrl_05_comprehensive_summary.png"))

    # Heatmaps (only if not filtered)
    if not args.planner and not args.estimator and not args.cbf:
        print("\n6. Controller-Estimator Heatmaps")
        plot_controller_estimator_heatmap(df_original, "mean_tracking_error",
                                         os.path.join(args.output_dir, "ctrl_06_heatmap_tracking_error.png"))
        print("  [OK] Saved: ctrl_06_heatmap_tracking_error.png")
        
        plot_controller_estimator_heatmap(df_original, "mean_velocity_change",
                                         os.path.join(args.output_dir, "ctrl_07_heatmap_smoothness.png"))
        print("  [OK] Saved: ctrl_07_heatmap_smoothness.png")

    print("\n7. Summary Table (CSV)")
    generate_summary_table(df, os.path.join(args.output_dir, "controller_summary_table.csv"))

    print("\n" + "="*80)
    print("SUCCESS: ALL VISUALIZATIONS GENERATED!")
    print("="*80)
    print(f"\nOutput directory: {os.path.abspath(args.output_dir)}")
    print(f"\nGenerated files:")
    for filename in sorted(os.listdir(args.output_dir)):
        if filename.startswith('ctrl_') and filename.endswith(('.png', '.csv')):
            print(f"  - {filename}")
    print("="*80)


if __name__ == "__main__":
    main()
