#!/usr/bin/env python3
"""
Analyze test results and generate comparison report
"""

import json
import os
import sys
from pathlib import Path
import statistics

def load_test_results(results_dir):
    """Load all JSON result files"""
    results = []
    
    for file in Path(results_dir).glob('*.json'):
        try:
            with open(file, 'r') as f:
                data = json.load(f)
                results.append(data)
        except Exception as e:
            print(f"Warning: Could not load {file}: {e}")
    
    return results

def analyze_results(results):
    """Analyze and compare results"""
    
    # Group by configuration
    configs = {}
    
    for result in results:
        name = result['test_name']
        # Extract planner, controller, cbf from name
        parts = name.split('_')
        if len(parts) >= 3:
            planner = parts[0]
            controller = parts[1]
            cbf = 'CBF' if 'cbf' in parts[2] else 'No CBF'
        else:
            continue
        
        config_key = f"{planner.upper()} + {controller.upper()} + {cbf}"
        
        if config_key not in configs:
            configs[config_key] = []
        
        configs[config_key].append(result)
    
    # Print comparison table
    print("\n" + "="*100)
    print("PERFORMANCE COMPARISON")
    print("="*100)
    print(f"{'Configuration':<30} {'Success':<10} {'Time (s)':<12} {'Path (m)':<12} {'Track Err':<12} {'Safety':<10}")
    print("-"*100)
    
    for config_name in sorted(configs.keys()):
        config_results = configs[config_name]
        
        # Compute averages
        success_rate = sum(1 for r in config_results if r['goal_reached']) / len(config_results) * 100
        
        successful_runs = [r for r in config_results if r['goal_reached']]
        
        if successful_runs:
            avg_time = statistics.mean(r['execution_time'] for r in successful_runs)
            avg_path = statistics.mean(r['path_length'] for r in successful_runs)
            avg_error = statistics.mean(r['mean_tracking_error'] for r in successful_runs)
            total_violations = sum(r['safety_violations'] for r in successful_runs)
            
            print(f"{config_name:<30} {success_rate:>6.0f}%    {avg_time:>8.2f}    {avg_path:>8.2f}    {avg_error:>8.4f}    {total_violations:>6}")
        else:
            print(f"{config_name:<30} {success_rate:>6.0f}%    {'N/A':<8}    {'N/A':<8}    {'N/A':<8}    {'N/A':<6}")
    
    print("="*100)
    print("\n")
    
    # Best configurations
    print("BEST CONFIGURATIONS:")
    print("-"*100)
    
    # Find best by execution time
    best_time_config = None
    best_time = float('inf')
    
    for config_name, config_results in configs.items():
        successful_runs = [r for r in config_results if r['goal_reached']]
        if successful_runs:
            avg_time = statistics.mean(r['execution_time'] for r in successful_runs)
            if avg_time < best_time:
                best_time = avg_time
                best_time_config = config_name
    
    if best_time_config:
        print(f"Fastest: {best_time_config} ({best_time:.2f}s)")
    
    # Find best by tracking error
    best_error_config = None
    best_error = float('inf')
    
    for config_name, config_results in configs.items():
        successful_runs = [r for r in config_results if r['goal_reached']]
        if successful_runs:
            avg_error = statistics.mean(r['mean_tracking_error'] for r in successful_runs)
            if avg_error < best_error:
                best_error = avg_error
                best_error_config = config_name
    
    if best_error_config:
        print(f"Best Tracking: {best_error_config} ({best_error:.4f}m)")
    
    # Find shortest path
    best_path_config = None
    best_path = float('inf')
    
    for config_name, config_results in configs.items():
        successful_runs = [r for r in config_results if r['goal_reached']]
        if successful_runs:
            avg_path = statistics.mean(r['path_length'] for r in successful_runs)
            if avg_path < best_path:
                best_path = avg_path
                best_path_config = config_name
    
    if best_path_config:
        print(f"Shortest Path: {best_path_config} ({best_path:.2f}m)")
    
    print("-"*100)
    print("\n")
    
    # CBF Analysis
    print("CBF IMPACT ANALYSIS:")
    print("-"*100)
    
    for planner in ['astar', 'rrtstar']:
        for controller in ['pid', 'lqr', 'mpc']:
            with_cbf_key = f"{planner.upper()} + {controller.upper()} + CBF"
            without_cbf_key = f"{planner.upper()} + {controller.upper()} + No CBF"
            
            if with_cbf_key in configs and without_cbf_key in configs:
                with_cbf = [r for r in configs[with_cbf_key] if r['goal_reached']]
                without_cbf = [r for r in configs[without_cbf_key] if r['goal_reached']]
                
                if with_cbf and without_cbf:
                    time_with = statistics.mean(r['execution_time'] for r in with_cbf)
                    time_without = statistics.mean(r['execution_time'] for r in without_cbf)
                    
                    violations_with = sum(r['safety_violations'] for r in with_cbf)
                    violations_without = sum(r['safety_violations'] for r in without_cbf)
                    
                    time_overhead = ((time_with - time_without) / time_without) * 100 if time_without > 0 else 0
                    
                    print(f"{planner.upper()}/{controller.upper()}:")
                    print(f"  Time overhead: {time_overhead:+.1f}%")
                    print(f"  Safety violations: {violations_without} -> {violations_with}")
    
    print("="*100)

def main():
    if len(sys.argv) > 1:
        results_dir = sys.argv[1]
    else:
        results_dir = "test_results"
    
    print(f"\nAnalyzing results from: {results_dir}")
    
    results = load_test_results(results_dir)
    
    if not results:
        print("No results found!")
        return
    
    print(f"Loaded {len(results)} test results\n")
    
    analyze_results(results)

if __name__ == '__main__':
    main()
