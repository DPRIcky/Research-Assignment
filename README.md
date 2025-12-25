# Autonomous Vehicle Navigation System

A comprehensive MATLAB implementation of autonomous vehicle navigation with multiple path planning algorithms, controllers, safety layers, and state estimation methods.

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage Modes](#usage-modes)
- [Components](#components)
- [Test Results](#test-results)
- [Configuration Options](#configuration-options)
- [File Structure](#file-structure)

## 🎯 Overview

This project implements a complete autonomous vehicle navigation system that integrates:
- **Path Planning**: A*, RRT*, Hybrid A*
- **Control**: PID, LQR, MPC
- **Safety**: Control Barrier Functions (CBF)
- **State Estimation**: Perfect, Noisy, EKF, External

The system supports **18 different configurations** (3 planners × 3 controllers × 2 CBF states) with 4 state estimation options.

## ✨ Features

### Path Planning Algorithms
1. **A\* (Grid-based)**
   - Deterministic search on occupancy grid
   - Manhattan distance heuristic
   - Fast and reliable for simple environments

2. **RRT\* (Sampling-based)**
   - Probabilistically complete
   - Rewiring for path optimization
   - Good for complex environments

3. **Hybrid A\* (Kinematic)**
   - Considers vehicle kinematics
   - Motion primitives with collision checking
   - Best for real vehicle constraints

### Controllers
1. **PID Controller**
   - Simple and interpretable
   - Tuned parameters: Kp_v=2.0, Kp_w=4.0
   - Good baseline performance

2. **LQR Controller**
   - Optimal linear control
   - State-space approach
   - Best overall performance (recommended)

3. **MPC Controller**
   - Predictive horizon (N=10)
   - Constraint handling
   - Computationally intensive

### Safety Layer
- **Control Barrier Functions (CBF)**
  - QP-based safety filter
  - Minimal intervention principle
  - Safety margin: 0.5m
  - Works with all controllers

### State Estimation
1. **Perfect**: Ground truth (ideal sensor)
2. **Noisy**: Gaussian noise added
3. **EKF**: Extended Kalman Filter
4. **External**: User-provided data

## 🏗️ System Architecture

```
┌─────────────┐     ┌──────────────┐     ┌────────────┐
│   State     │────▶│     Path     │────▶│ Controller │
│ Estimator   │     │   Planner    │     │            │
└─────────────┘     └──────────────┘     └────────────┘
                                                │
                                                ▼
                    ┌──────────────┐     ┌────────────┐
                    │   Vehicle    │◀────│    CBF     │
                    │   Dynamics   │     │   Safety   │
                    └──────────────┘     └────────────┘
```

## 📦 Installation

### Requirements
- MATLAB R2023a or later
- Optimization Toolbox (for MPC and CBF)
- No additional toolboxes required

### Setup
```bash
git clone <repository-url>
cd "Reseaarch Assignment 26 Dec, 2025/matlab"
```

No installation needed - all code is self-contained.

## 🚀 Quick Start

### Method 1: Batch Mode (Fastest)
```matlab
% Run with Hybrid A* + LQR + CBF + Perfect estimation
cd matlab
matlab -batch "planner=3; controller=2; cbf_enabled=1; estimator=1; run('run_simulation.m');"
```

### Method 2: Interactive Mode
```matlab
% Run interactive script
cd matlab
matlab -r "run_simulation_interactive"

% Follow prompts:
% Select planner (1-3): 3
% Select controller (1-3): 2
% Enable CBF safety? (y/n): y
% Select state estimator (1-4): 1
```

### Method 3: Comprehensive Testing
```matlab
% Run full test suite (6 configurations)
cd matlab
matlab -batch "run('run_comprehensive_tests.m');"
```

## 📖 Usage Modes

### 1. Single Configuration Test
```matlab
% Set parameters
planner = 3;        % 1=A*, 2=RRT*, 3=Hybrid A*
controller = 2;     % 1=PID, 2=LQR, 3=MPC
cbf_enabled = 1;    % 0=disabled, 1=enabled
estimator = 1;      % 1=Perfect, 2=Noisy, 3=EKF, 4=External

% Run simulation
run('run_simulation.m');
```

### 2. Interactive Selection
```matlab
run_simulation_interactive
```
This will prompt you to select each component interactively.

### 3. Batch Testing
```matlab
run_comprehensive_tests
```
Tests 6 key configurations and generates comparison plots.

## 🧩 Components

### Path Planners (`planners/`)

#### A* Planner
```matlab
planner = AStarPlanner(resolution, heuristic_weight);
path = planner.plan(environment, start, goal);
```
- **Resolution**: 1.0m grid spacing
- **Heuristic**: Manhattan distance
- **Success Rate**: ~95%

#### RRT* Planner
```matlab
planner = RRTStarPlanner(max_iterations, step_size, goal_sample_rate, search_radius);
path = planner.plan(environment, start, goal);
```
- **Max Iterations**: 3000
- **Step Size**: 2.0m
- **Goal Sample Rate**: 10%
- **Search Radius**: 5.0m

#### Hybrid A* Planner
```matlab
planner = HybridAStarPlanner(resolution, num_headings);
path = planner.plan(environment, start, goal);
```
- **Resolution**: 1.0m
- **Headings**: 16 directions
- **Motion Primitives**: Forward/backward arcs
- **Best Performance**: Kinematic feasibility

### Controllers (`controllers/`)

#### PID Controller
```matlab
ctrl = PIDController(Kp_v, Ki_v, Kd_v, Kp_w, Ki_w, Kd_w);
[v, w, ctrl] = ctrl.track_path(state, path, lookahead, dt);
```
- **Linear Gains**: Kp=2.0, Ki=0.2, Kd=1.0
- **Angular Gains**: Kp=4.0, Ki=0.2, Kd=0.6
- **Lookahead**: 3.0m

#### LQR Controller
```matlab
Q = diag([10, 10, 5, 1]);  % State weights
R = diag([1, 10]);          % Control weights
ctrl = LQRController(Q, R, dt, 'bicycle', L);
[v, w] = ctrl.track_path(state, path, lookahead);
```
- **State Weights**: Position (10), heading (5), velocity (1)
- **Control Weights**: Linear (1), angular (10)
- **Lookahead**: 3.5m
- **Recommended**: Best overall performance

#### MPC Controller
```matlab
N = 10;  % Prediction horizon
ctrl = MPCController(N, dt, Q, R, Qf, 'bicycle', L);
[v, w] = ctrl.track_path(state, path, lookahead);
```
- **Horizon**: 10 steps (1 second)
- **Terminal Cost**: Qf = 2×Q
- **Computation**: ~100-500ms per step

### Safety Layer (`safety/`)

#### CBF Safety Filter
```matlab
cbf = CBFSafetyFilter(environment, safety_margin, alpha, model_type, L, dt);
[v_safe, w_safe, modified] = cbf.filter_control(state, v_nom, w_nom);
```
- **Safety Margin**: 0.5m (buffer around obstacles)
- **Alpha**: 0.3 (barrier function decay rate)
- **Method**: Quadratic Programming (QP)
- **Minimal Intervention**: Only modifies control when necessary

### State Estimation (`utils/`)

#### StateEstimator (Perfect/Noisy/External)
```matlab
est = StateEstimator('perfect');           % Ground truth
est = StateEstimator('noisy', [0.5, 0.5, 0.1]);  % Gaussian noise
est = StateEstimator('external');          % External data
state_est = est.get_state(true_state, t);
```

#### EKF Estimator
```matlab
Q = diag([0.1, 0.1, 0.05, 0.1]);      % Process noise
R = diag([0.5, 0.5, 0.1]);             % Measurement noise
ekf = EKFEstimator(dt, Q, R, L);
ekf = ekf.initialize([x, y, theta, v]);
ekf = ekf.predict([a; delta]);
[ekf, state_est] = ekf.update([x_meas; y_meas; theta_meas]);
```

## 📊 Test Results

### Recommended Configurations

| Configuration | Goal Distance | Tracking Error | CBF Active | Min Obs Dist | Status |
|---------------|---------------|----------------|------------|--------------|--------|
| **Hybrid A* + LQR + CBF** | 0.960 m | 0.501 m | 37.2% | 1.531 m | ✓ **BEST** |
| **A* + LQR + CBF** | 0.911 m | 0.449 m | 70.4% | 0.501 m | ✓ Good |
| **Hybrid A* + LQR (No CBF)** | 0.969 m | 0.501 m | N/A | N/A | ✓ Fast |

### Performance Analysis

**Best Overall**: Hybrid A* + LQR + CBF
- ✅ Reaches goal (< 1m error)
- ✅ Smooth tracking (0.5m mean error)
- ✅ Safe navigation (1.5m+ clearance)
- ✅ Moderate CBF intervention (37%)
- ✅ Fast computation (< 2s)

**Known Issues**:
- MPC: Too conservative with CBF (gets stuck)
- EKF: Needs better tuning (large estimation errors)
- RRT*: Occasional planning failures

## ⚙️ Configuration Options

### Planner Selection
```matlab
planner = 1;  % A* - Fast, grid-based
planner = 2;  % RRT* - Sampling-based (may fail)
planner = 3;  % Hybrid A* - Kinematic (recommended)
```

### Controller Selection
```matlab
controller = 1;  % PID - Simple baseline
controller = 2;  % LQR - Best performance (recommended)
controller = 3;  % MPC - Advanced but slow
```

### CBF Safety
```matlab
cbf_enabled = 0;  % Disabled - Faster, no safety guarantees
cbf_enabled = 1;  % Enabled - Safe, minimal intervention
```

### State Estimation
```matlab
estimator = 1;  % Perfect - Ground truth (testing)
estimator = 2;  % Noisy - Realistic sensors
estimator = 3;  % EKF - Kalman filtering (needs tuning)
estimator = 4;  % External - User-provided data
```

## 📁 File Structure

```
matlab/
├── run_simulation.m              # Main simulation (batch mode)
├── run_simulation_interactive.m  # Interactive user prompts
├── run_comprehensive_tests.m     # Automated testing suite
│
├── planners/
│   ├── AStarPlanner.m           # A* algorithm
│   ├── RRTStarPlanner.m         # RRT* algorithm
│   ├── HybridAStarPlanner.m     # Hybrid A* algorithm
│   ├── test_astar.m             # A* tests
│   ├── test_rrt_star.m          # RRT* tests
│   └── test_hybrid_astar.m      # Hybrid A* tests
│
├── controllers/
│   ├── PIDController.m          # PID controller
│   ├── LQRController.m          # LQR controller
│   ├── MPCController.m          # MPC controller
│   ├── test_pid.m               # PID tests
│   ├── test_lqr.m               # LQR tests
│   └── test_mpc.m               # MPC tests
│
├── safety/
│   ├── CBFSafetyFilter.m        # CBF safety layer
│   └── test_phase4.m            # CBF integration tests
│
└── utils/
    ├── Environment.m            # Obstacle map
    ├── VehicleDynamics.m        # Bicycle/differential model
    ├── StateEstimator.m         # Basic state estimation
    └── EKFEstimator.m           # Extended Kalman Filter
```

## 🎨 Output Visualization

Each simulation generates a 6-panel figure:

1. **Trajectory Plot**: Planned path vs actual trajectory
2. **Tracking Error**: Distance from path over time
3. **Estimation Error**: State estimation accuracy
4. **Linear Velocity**: Control input v(t)
5. **Angular Velocity**: Control input ω(t)
6. **Safety Monitor**: CBF activation and obstacle distances

Files saved:
- `results_<planner>_<controller>_cbf<0|1>_est<1-4>.png`
- `test_results.mat` (from comprehensive tests)
- `comprehensive_test_results.png` (comparison plots)

## 🔧 Tuning Guide

### Controller Tuning

**PID**:
- Increase Kp_v for faster speed response
- Increase Kp_w for tighter heading control
- Add Ki for steady-state error elimination

**LQR**:
- Increase Q(1:2) for better position tracking
- Increase Q(3) for better heading tracking
- Increase R for smoother control

**MPC**:
- Increase N for longer horizon (more computation)
- Adjust Q/R trade-off
- Set tighter constraints if needed

### CBF Tuning

```matlab
safety_margin = 0.5;  % Increase for more caution
alpha = 0.3;          % Increase for stronger enforcement
```

### EKF Tuning

```matlab
% Process noise (vehicle model uncertainty)
Q = diag([0.1, 0.1, 0.05, 0.1]);

% Measurement noise (sensor accuracy)
R = diag([0.5, 0.5, 0.1]);
```

## 📝 Example Usage

### Example 1: Safe Navigation
```matlab
% Safest configuration - reaches goal with high clearance
planner = 3;        % Hybrid A* (smooth paths)
controller = 2;     % LQR (optimal tracking)
cbf_enabled = 1;    % CBF enabled (safety)
estimator = 1;      % Perfect (no noise)
run('run_simulation.m');
```

### Example 2: Realistic Scenario
```matlab
% Real-world with noisy sensors
planner = 3;
controller = 2;
cbf_enabled = 1;
estimator = 2;      % Noisy measurements
run('run_simulation.m');
```

### Example 3: Performance Testing
```matlab
% Fastest execution (no safety layer)
planner = 3;
controller = 2;
cbf_enabled = 0;    % No CBF (faster)
estimator = 1;
run('run_simulation.m');
```

## 🐛 Troubleshooting

### "Path planning failed"
- Try different planner (RRT* can fail occasionally)
- Check obstacle placement
- Increase RRT* iterations

### "Robot doesn't reach goal"
- Check CBF is not too conservative
- Verify controller tuning
- Try LQR controller (most reliable)

### "Collision detected"
- Enable CBF safety layer
- Increase safety margin
- Use Hybrid A* for better paths

### "Simulation too slow"
- Disable MPC (use LQR instead)
- Disable CBF for faster execution
- Reduce simulation time T

## 📚 References

- **A\***: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968)
- **RRT\***: Karaman, S., & Frazzoli, E. (2011)
- **Hybrid A\***: Dolgov, D., et al. (2008)
- **LQR**: Anderson, B. D., & Moore, J. B. (1990)
- **MPC**: Camacho, E. F., & Bordons, C. (2007)
- **CBF**: Ames, A. D., et al. (2017)
- **EKF**: Welch, G., & Bishop, G. (1995)

## 👥 Authors

Research Assignment - December 26, 2025

## 📄 License

Academic use only.

---

**Note**: For best results, use **Hybrid A* + LQR + CBF + Perfect** configuration. This provides optimal balance of performance, safety, and computational efficiency.
