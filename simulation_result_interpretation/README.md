# Simulation Result Interpretation (Section 3.1)

This folder contains scripts to generate planning-performance plots for the report.

## Expected files
Copy the following into `metrics` (or point --data-dir to the folder):
- *_metrics.csv
- *_trajectory.csv

File naming format:
- {Planner}_{Controller}_CBF{0|1}_{Estimator}_metrics.csv
- {Planner}_{Controller}_CBF{0|1}_{Estimator}_trajectory.csv

Examples:
- AStar_LQR_CBF1_EKF_metrics.csv
- RRTStar_MPC_CBF0_Perfect_trajectory.csv

## Requirements
Python 3 with:
- pandas
- matplotlib

Install if needed:
  pip install pandas matplotlib

## Generate planning plots
From this folder:
  python planning_performance_plots.py --data-dir metrics --estimator EKF --cbf ON

Outputs (saved to ./plots):
- planning_path_length.png
- planning_time.png
- planning_smoothness.png
- planning_success_rate.png

## Example trajectories plot
  python example_trajectories.py --data-dir metrics --estimator EKF --cbf ON

Output:
- example_trajectories.png

## Optional filters
- --estimator EKF | PF | Perfect | Noisy | UKF
- --cbf ON | OFF
- --controller PID | LQR | MPC (only for example_trajectories.py)
