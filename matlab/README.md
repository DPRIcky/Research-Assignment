# Autonomous Vehicle Navigation Simulation - MATLAB Package

## Overview

This MATLAB package implements a comprehensive autonomous vehicle navigation system with path planning, trajectory tracking control, and safety monitoring capabilities. The system integrates multiple planning algorithms, control strategies, and state estimation methods.

## Requirements

- MATLAB R2020b or later
- Optimization Toolbox (for MPC and CBF QP solvers)
- Control System Toolbox (for LQR)

## Installation

1. Clone or download this repository
2. Add the MATLAB package to your path:
```matlab
cd /path/to/matlab
addpath(genpath(pwd))
```

## Quick Start

### Interactive Simulation

Run the main interactive simulation:
```matlab
cd matlab
main_simulation
```

This will prompt you to select:
1. Path planner (A*, RRT*, Hybrid A*)
2. Controller (PID, LQR, MPC)
3. CBF safety filter (Enable/Disable)
4. State estimator (Perfect, Noisy, EKF, UKF, PF, External)

### Automated Testing

Run comprehensive tests across all configurations:
```matlab
run_comprehensive_tests
```

### Individual Component Tests

Test individual components:
```matlab
test_phase1          % Path planners
test_phase2_part1    % PID controller
test_phase2_part2    % LQR controller
test_phase2_part3    % MPC controller
test_phase3_part1    % CBF safety filter
test_estimators      % State estimators (EKF, UKF, PF)
```

## Package Structure

```
matlab/
├── core/
│   ├── Environment.m           % Environment with obstacles
│   └── VehicleDynamics.m       % Bicycle model dynamics
├── planners/
│   ├── AStarPlanner.m          % A* grid-based planner
│   ├── RRTStarPlanner.m        % RRT* sampling-based planner
│   └── HybridAStarPlanner.m    % Hybrid A* kinematic planner
├── controllers/
│   ├── PIDController.m         % PID trajectory tracker
│   ├── LQRController.m         % LQR optimal controller
│   └── MPCController.m         % MPC predictive controller
├── safety/
│   └── CBFSafetyFilter.m       % Control Barrier Function filter
├── utils/
│   ├── StateEstimator.m        % Base state estimator
│   ├── EKFEstimator.m          % Extended Kalman Filter
│   ├── UKFEstimator.m          % Unscented Kalman Filter
│   ├── ParticleFilter.m        % Particle Filter (SIR)
│   ├── compute_metrics.m       % Performance metrics computation
│   ├── plot_simulation_results.m  % Visualization utilities
│   └── save_results.m          % Results export (MAT/CSV)
├── main_simulation.m           % Main interactive simulation
├── run_comprehensive_tests.m   % Automated testing suite
└── test_*.m                    % Individual component tests
```

## Components

### Path Planners

#### A* Planner
- **Algorithm**: Grid-based search with Manhattan heuristic
- **Pros**: Complete, optimal on grid
- **Cons**: Resolution-dependent, no kinematic constraints
- **Usage**: Fast planning in structured environments

#### RRT* Planner
- **Algorithm**: Sampling-based with asymptotic optimality
- **Pros**: Probabilistically complete, handles complex spaces
- **Cons**: Non-deterministic, slower convergence
- **Usage**: Complex obstacle environments

#### Hybrid A* Planner
- **Algorithm**: Grid search with kinematic constraints
- **Pros**: Kinematically feasible paths
- **Cons**: Higher computational cost
- **Usage**: Realistic vehicle motion planning

### Controllers

#### PID Controller
- **Type**: Proportional-Integral-Derivative
- **Gains**: Velocity (Kp=4.0, Ki=0.8, Kd=2.0), Heading (Kp=8.0, Ki=1.0, Kd=1.5)
- **Pros**: Simple, fast, no model required
- **Cons**: Limited performance with complex paths

#### LQR Controller
- **Type**: Linear Quadratic Regulator
- **Weights**: Q=diag([10,10,5,1]), R=diag([1,10])
- **Pros**: Optimal for linearized system, smooth control
- **Cons**: Requires model, local linearization

#### MPC Controller
- **Type**: Model Predictive Control
- **Horizon**: N=20 steps
- **Weights**: Q=diag([40,40,20,4]), R=diag([0.05,0.5])
- **Pros**: Handles constraints, predictive
- **Cons**: Computationally expensive

### Safety Layer

#### CBF (Control Barrier Function)
- **Function**: Real-time safety filter using QP optimization
- **Features**:
  - Collision avoidance
  - Minimum distance constraints
  - Minimal control modification
- **Parameters**:
  - Safety margin: 0.3m (reactive) or 1.0m (MPC)
  - Alpha parameter: 0.5 (aggressive) or 2.0 (relaxed)

### State Estimators

#### Perfect
- Ground truth state (ideal sensor)

#### Noisy
- Ground truth + Gaussian noise (σ_pos = 0.5m, σ_θ = 0.1rad)

#### EKF (Extended Kalman Filter)
- Linearization-based estimation
- Process noise: diag([0.1, 0.1, 0.05, 0.1])
- Measurement noise: diag([0.5, 0.5, 0.1])

#### UKF (Unscented Kalman Filter)
- Unscented transform with sigma points
- Better handling of nonlinearities than EKF

#### PF (Particle Filter)
- Sequential Importance Resampling
- 500 particles
- Handles highly nonlinear systems

#### External
- Load pre-recorded state estimates from file

## Environment Configuration

Default environment:
- **Size**: 50m × 50m
- **Obstacles**: 3 circular obstacles
  - Obstacle 1: Center (25, 20), Radius 5m
  - Obstacle 2: Center (30, 30), Radius 3m
  - Obstacle 3: Center (15, 35), Radius 4m
- **Start**: (5, 5, 0)
- **Goal**: (45, 45, π/4)

## Performance Metrics

The package computes comprehensive metrics:

### Planning Metrics
- Path length
- Planning time
- Nodes explored
- Success rate

### Control Metrics
- Mean/max tracking error
- RMS tracking error
- Control smoothness
- Path efficiency

### Safety Metrics
- CBF activation rate
- Minimum obstacle distance
- Safety violations
- Safety maintenance

### Smoothness Metrics
- Curvature (heading changes)
- Jerk (acceleration changes)
- Control input variations

## Output Files

Results are automatically saved to `matlab/results/`:
- `*.mat` - MATLAB workspace with all variables
- `*_metrics.csv` - Performance metrics table
- `*_trajectory.csv` - Time-series trajectory data
- `*_config.txt` - Simulation configuration

## Examples

### Example 1: A* + LQR + CBF + EKF
```matlab
% Interactive mode
main_simulation
% Select: 1 (A*), 2 (LQR), 1 (CBF=Yes), 3 (EKF)
```

### Example 2: Hybrid A* + MPC + No CBF + Perfect
```matlab
% Interactive mode
main_simulation
% Select: 3 (Hybrid A*), 3 (MPC), 0 (CBF=No), 1 (Perfect)
```

### Example 3: Programmatic Execution
```matlab
% See run_comprehensive_tests.m for examples of programmatic execution
```

## Tuning Guidelines

### PID Controller
- Increase Kp for faster response (may cause oscillation)
- Increase Kd for damping
- Increase Ki for steady-state error reduction

### LQR Controller
- Increase Q weights to penalize state errors
- Increase R weights to reduce control effort

### MPC Controller
- Increase horizon N for better long-term planning (slower)
- Tune Q/R balance for tracking vs. smoothness trade-off
- Adjust lookahead distance for path following

### CBF Safety
- Increase safety_margin for more conservative behavior
- Increase alpha for less aggressive intervention
- Use adaptive parameters: tighter for reactive controllers, looser for MPC

## Troubleshooting

### Planning fails
- Check obstacle configuration
- Verify start/goal are collision-free
- Increase grid resolution (A*) or samples (RRT*)

### Controller diverges
- Reduce velocity limits
- Tune control gains
- Check path smoothness

### CBF too conservative
- Increase alpha parameter
- Increase safety margin
- Disable CBF near goal region

### Estimator diverges
- Check process/measurement noise covariances
- Verify control input format (EKF uses [a,δ], UKF/PF use [v,ω])
- Increase measurement frequency

## License

This project is licensed under the MIT License.

## Citation

If you use this code in your research, please cite:

```
@software{autonomous_nav_matlab,
  title = {Autonomous Vehicle Navigation Simulation - MATLAB Package},
  author = {Your Name},
  year = {2025},
  url = {https://github.com/yourusername/autonomous-nav}
}
```

## Contact

For questions or issues, please contact: your.email@example.com

## References

1. LaValle, S. M. (2006). Planning Algorithms. Cambridge University Press.
2. Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. IJRR.
3. Ames, A. D., et al. (2019). Control barrier functions: Theory and applications. ECC.
4. Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics. MIT Press.
