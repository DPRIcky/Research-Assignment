# Quick Usage Guide

## 🚀 Three Ways to Run the Simulation

### 1. Interactive Mode (Easiest for Beginners)
```bash
cd matlab
matlab -r "run_simulation_interactive"
```
**What happens:**
- You'll be prompted to select each component
- Choose planner (1-3)
- Choose controller (1-3)
- Enable/disable CBF (y/n)
- Choose state estimator (1-4)

### 2. Batch Mode (Fastest)
```bash
cd matlab
# Run best configuration
matlab -batch "planner=3; controller=2; cbf_enabled=1; estimator=1; run('run_simulation.m');"
```

**Configuration codes:**
- **Planner:** 1=A*, 2=RRT*, 3=Hybrid A* (recommended)
- **Controller:** 1=PID, 2=LQR (recommended), 3=MPC
- **CBF:** 0=disabled, 1=enabled (recommended)
- **Estimator:** 1=Perfect, 2=Noisy, 3=EKF, 4=External

### 3. Quick Start Script (Most Convenient)
```bash
./quick_start.sh
```
Select from menu:
1. Interactive mode
2. Best configuration (Hybrid A* + LQR + CBF)
3. Fast configuration (no CBF)
4. Comprehensive test suite
5. Custom configuration

---

## 📊 Testing and Analysis

### Run Comprehensive Tests
```bash
cd matlab
matlab -batch "run('run_comprehensive_tests.m');"
```
This runs 6 configurations and generates comparison plots.

### View Previous Results
```matlab
cd matlab
matlab -r "view_results"
```
Loads and displays results from comprehensive tests.

---

## 🎯 Recommended Configurations

### Best Overall Performance
```matlab
planner = 3;        % Hybrid A*
controller = 2;     % LQR
cbf_enabled = 1;    % Safety on
estimator = 1;      % Perfect
run('run_simulation.m');
```
**Expected:** Goal distance < 1m, Safe navigation

### Fastest Execution
```matlab
planner = 3;        % Hybrid A*
controller = 2;     % LQR
cbf_enabled = 0;    % Safety off (faster)
estimator = 1;      % Perfect
run('run_simulation.m');
```
**Expected:** ~0.6s execution time

### Realistic Scenario
```matlab
planner = 3;        % Hybrid A*
controller = 2;     % LQR
cbf_enabled = 1;    % Safety on
estimator = 2;      % Noisy sensors
run('run_simulation.m');
```
**Expected:** Performance with sensor noise

---

## 📁 Important Files

### Main Simulation Files
- `run_simulation.m` - Main simulation (batch mode)
- `run_simulation_interactive.m` - Interactive user prompts
- `run_comprehensive_tests.m` - Automated testing
- `view_results.m` - Results viewer

### Testing Files
- `test_phase2_part1.m` - A* planner tests
- `test_phase2_part2.m` - RRT* planner tests
- `test_phase2_part3.m` - Hybrid A* planner tests
- `test_phase3_part1.m` - PID controller tests
- `test_phase3_part2.m` - LQR controller tests
- `test_phase3_part3.m` - MPC controller tests
- `test_phase4.m` - CBF safety tests

### Documentation
- `README.md` - Complete user guide
- `IMPLEMENTATION_SUMMARY.md` - Technical details
- `PROJECT_STATUS.md` - Project completion status
- `USAGE_GUIDE.md` - This file

---

## 🔍 Output Files

After running simulations, you'll find:
- `results_*.png` - Trajectory and performance plots
- `test_results.mat` - Test data (from comprehensive tests)
- `comprehensive_test_results.png` - Comparison plots

---

## 💡 Tips

### For Best Results
1. Use **Hybrid A* + LQR + CBF** combination
2. Start with **Perfect** estimation to verify
3. Enable **CBF** for safety guarantees
4. Run **comprehensive tests** to compare

### For Fastest Results
1. Use **Hybrid A* + LQR** (no CBF)
2. Keep estimator as **Perfect**
3. Use **batch mode** execution

### For Testing
1. Run individual test files first
2. Then run comprehensive tests
3. Use `view_results` to analyze
4. Check generated PNG files

---

## 🐛 Common Issues

### "Path planning failed"
- Try different planner (use Hybrid A* instead of RRT*)

### "Robot doesn't reach goal"
- Check if MPC is selected (known issue with CBF)
- Try LQR controller instead

### "MATLAB command not found"
- Make sure MATLAB is in your PATH
- Or use full path: `/path/to/matlab -batch "..."`

---

## 📞 Quick Reference

### One-Line Commands

**Best configuration:**
```bash
cd matlab && matlab -batch "planner=3; controller=2; cbf_enabled=1; estimator=1; run('run_simulation.m');"
```

**Fast configuration:**
```bash
cd matlab && matlab -batch "planner=3; controller=2; cbf_enabled=0; estimator=1; run('run_simulation.m');"
```

**Run all tests:**
```bash
cd matlab && matlab -batch "run('run_comprehensive_tests.m');"
```

**View results:**
```bash
cd matlab && matlab -r "view_results; exit"
```

---

**For detailed information, see [README.md](README.md)**
