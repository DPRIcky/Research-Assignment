# File Manifest - MATLAB Autonomous Vehicle Navigation Package

## Core Components

### Environment and Dynamics

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `core/Environment.m` | Defines navigation environment with obstacles | Size [width, height], resolution | Environment object with obstacle list and collision checking methods |
| `core/VehicleDynamics.m` | Bicycle model vehicle dynamics | Model type ('bicycle'), dt | Vehicle object with step() method for state propagation |

## Path Planning

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `planners/AStarPlanner.m` | Grid-based A* path planning | Environment, resolution | Path (Nx2: x,y), statistics (success, length, time, nodes) |
| `planners/RRTStarPlanner.m` | Sampling-based RRT* planning | Environment, resolution | Path (Nx2: x,y), statistics (success, length, time, nodes) |
| `planners/HybridAStarPlanner.m` | Kinematic-feasible Hybrid A* | Environment, resolution, wheelbase | Path (Nx3: x,y,θ), statistics (success, length, time, nodes) |

## Controllers

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `controllers/PIDController.m` | PID trajectory tracking controller | State (4x1), path (Nx3), lookahead, dt | Linear velocity (v), angular velocity (ω), updated controller state |
| `controllers/LQRController.m` | LQR optimal trajectory tracking | State (4x1), path (Nx3), lookahead | Linear velocity (v), angular velocity (ω) |
| `controllers/MPCController.m` | MPC predictive trajectory tracking | State (4x1), path (Nx3), lookahead | Linear velocity (v), angular velocity (ω) |

**Control Outputs:**
- All controllers output: `[v, ω]` where v is linear velocity (m/s) and ω is angular velocity (rad/s)

## Safety Layer

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `safety/CBFSafetyFilter.m` | Control Barrier Function safety filter | State (4x1), nominal controls (v,ω) | Safe controls (v_safe, ω_safe), modification flag |

**Features:**
- Real-time QP optimization for minimal control modification
- Obstacle avoidance constraints
- Configurable safety margins and aggressiveness (alpha parameter)

## State Estimation

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `utils/StateEstimator.m` | Base state estimator (perfect/noisy) | True state (4x1), time | Estimated state (4x1) |
| `utils/EKFEstimator.m` | Extended Kalman Filter | Control [a,δ], measurement [x,y,θ] | Estimated state (4x1), covariance (4x4) |
| `utils/UKFEstimator.m` | Unscented Kalman Filter | Control [v,ω], measurement [x,y,θ] | Estimated state (4x1), covariance (4x4) |
| `utils/ParticleFilter.m` | Particle Filter (SIR) | Control [v,ω], measurement [x,y,θ] | Estimated state (4x1), particles (Nx4) |

**State Vector:** `[x, y, θ, v]` - position (x,y), heading (θ), velocity (v)

**Control Formats:**
- EKF: `[a, δ]` - acceleration, steering angle
- UKF/PF: `[v, ω]` - linear velocity, angular velocity

## Utilities

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `utils/compute_metrics.m` | Calculate performance metrics | Trajectory, path, controls, timestamps, CBF data, planner stats | Metrics struct with 30+ performance indicators |
| `utils/plot_simulation_results.m` | Generate visualization plots | Environment, trajectories, controls, errors, config | Figure handle with 6-panel visualization |
| `utils/save_results.m` | Export results to files | Metrics, trajectory, controls, config, filename | MAT file, CSV files (metrics + trajectory), config TXT |

**Computed Metrics:**
- Planning: path length, planning time, nodes explored, success rate
- Tracking: mean/max/RMS tracking error, final error
- Smoothness: curvature, jerk, control changes
- Safety: CBF activations, minimum distances, violations
- Efficiency: path efficiency, average velocity

## Main Scripts

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `main_simulation.m` | Interactive simulation with user selection | User prompts for planner/controller/CBF/estimator | Complete simulation results with visualization |
| `run_comprehensive_tests.m` | Automated testing suite for all combinations | None (automated) | Test results for all 108 configurations |
| `run_simulation.m` | Legacy simulation script | None | Basic simulation results |

## Test Scripts

| File | Purpose | Inputs | Outputs |
|------|---------|--------|---------|
| `test_phase1.m` | Test all path planners | None | Planner comparison visualization |
| `test_phase2_part1.m` | Test PID controller | None | PID performance metrics |
| `test_phase2_part2.m` | Test LQR controller | None | LQR performance metrics |
| `test_phase2_part3.m` | Test MPC controller | None | MPC performance metrics |
| `test_phase3_part1.m` | Test CBF safety filter | None | CBF safety demonstration |
| `test_phase3_part2.m` | Test CBF with different planners | None | CBF-planner integration |
| `test_phase3_part3.m` | Test CBF with different controllers | None | CBF-controller integration |
| `test_phase4.m` | Test complete integration | None | End-to-end system validation |
| `test_estimators.m` | Test EKF/UKF/PF estimators | None | Estimator comparison on figure-8 trajectory |

## Configuration Files

| File | Purpose | Contents |
|------|---------|----------|
| `README.md` | Package documentation | Installation, usage, examples, troubleshooting |
| `LICENSE` | MIT License | Open-source license terms |
| `FILE_MANIFEST.md` | This file | Complete file listing with descriptions |

## Output Directories

### `results/`
Generated during simulation runs. Contains:
- `*.mat` - MATLAB workspace files with complete simulation data
- `*_metrics.csv` - Performance metrics in CSV format
- `*_trajectory.csv` - Time-series trajectory data
- `*_config.txt` - Configuration parameters used

## Data Flow

### Main Simulation Loop

```
1. User Selection
   ├── Planner: A*/RRT*/Hybrid A*
   ├── Controller: PID/LQR/MPC
   ├── CBF: Enabled/Disabled
   └── Estimator: Perfect/Noisy/EKF/UKF/PF/External

2. Initialization
   ├── Environment.m → Create obstacles
   ├── VehicleDynamics.m → Setup vehicle model
   ├── Planner → Generate reference path
   ├── Controller → Initialize controller
   ├── CBFSafetyFilter.m → Setup safety constraints (if enabled)
   └── Estimator → Initialize state estimate

3. Simulation Loop (dt = 0.1s)
   ├── True State → VehicleDynamics.step()
   ├── Measurement → Add sensor noise
   ├── Estimator → predict() + update() → Estimated State
   ├── Controller → track_path() → Nominal Control [v, ω]
   ├── CBF → filter_control() → Safe Control [v_safe, ω_safe] (if enabled)
   └── Apply control → Update state

4. Post-Processing
   ├── compute_metrics.m → Performance metrics
   ├── plot_simulation_results.m → Visualization
   └── save_results.m → Export data

5. Output
   ├── Console: Performance summary
   ├── Figures: 6-panel visualization
   └── Files: MAT + CSV + TXT in results/
```

## Dependencies

### External Toolboxes
- **Optimization Toolbox**: Required for `quadprog()` in CBFSafetyFilter and `fmincon()` in MPCController
- **Control System Toolbox**: Required for `lqr()` in LQRController

### Internal Dependencies
```
main_simulation.m
├── core/Environment.m
├── core/VehicleDynamics.m
├── planners/{AStarPlanner, RRTStarPlanner, HybridAStarPlanner}.m
├── controllers/{PIDController, LQRController, MPCController}.m
├── safety/CBFSafetyFilter.m (optional)
├── utils/{StateEstimator, EKFEstimator, UKFEstimator, ParticleFilter}.m
├── utils/compute_metrics.m
├── utils/plot_simulation_results.m
└── utils/save_results.m
```

## Configuration Parameters

### Default Environment
- **Size**: 50m × 50m
- **Resolution**: 1.0m (planners)
- **Obstacles**: 
  - (25, 20, r=5)
  - (30, 30, r=3)
  - (15, 35, r=4)
- **Start**: (5, 5, 0°)
- **Goal**: (45, 45, 45°)

### Vehicle Model (Bicycle)
- **Wheelbase**: L = 2.5m
- **Max Velocity**: 5.0 m/s
- **Max Steering**: ±45°
- **Time Step**: dt = 0.1s

### Controller Tuning
- **PID**: Kp_v=4.0, Ki_v=0.8, Kd_v=2.0, Kp_θ=8.0, Ki_θ=1.0, Kd_θ=1.5
- **LQR**: Q=diag([10,10,5,1]), R=diag([1,10])
- **MPC**: N=20, Q=diag([40,40,20,4]), R=diag([0.05,0.5]), Qf=diag([100,100,50,10])

### CBF Safety
- **Reactive (PID/LQR)**: margin=0.3m, alpha=0.5
- **Predictive (MPC)**: margin=1.0m, alpha=2.0

### State Estimation
- **Process Noise**: diag([0.1, 0.1, 0.05, 0.1])
- **Measurement Noise**: diag([0.5, 0.5, 0.1])
- **Particle Filter**: N_particles = 500

## Version History

- **v1.0** (2025-12-26): Initial release
  - 3 planners (A*, RRT*, Hybrid A*)
  - 3 controllers (PID, LQR, MPC)
  - CBF safety layer
  - 6 state estimation modes
  - Comprehensive metrics and visualization
  - Export to MAT/CSV

## Total File Count

- **Core**: 2 files
- **Planners**: 3 files
- **Controllers**: 3 files
- **Safety**: 1 file
- **Estimators**: 4 files
- **Utilities**: 3 files
- **Main Scripts**: 3 files
- **Test Scripts**: 9 files
- **Documentation**: 3 files

**Total: 31 MATLAB files + 3 documentation files**
