#!/bin/bash
# Quick Start Script for Autonomous Vehicle Navigation System

echo "========================================================="
echo "  Autonomous Vehicle Navigation - Quick Start"
echo "========================================================="
echo ""

# Change to matlab directory
cd "$(dirname "$0")/matlab" || exit 1

echo "Select a run mode:"
echo "  1. Interactive Mode (user prompts)"
echo "  2. Batch Mode - Best Configuration (Hybrid A* + LQR + CBF)"
echo "  3. Batch Mode - Fast (No CBF)"
echo "  4. Comprehensive Test Suite (6 configurations)"
echo "  5. Custom Configuration"
echo ""
read -p "Enter choice (1-5): " choice

case $choice in
    1)
        echo ""
        echo "Starting interactive mode..."
        echo "You will be prompted to select:"
        echo "  - Path planner (A*/RRT*/Hybrid A*)"
        echo "  - Controller (PID/LQR/MPC)"
        echo "  - CBF safety (enabled/disabled)"
        echo "  - State estimator (Perfect/Noisy/EKF/External)"
        echo ""
        matlab -r "run_simulation_interactive; exit"
        ;;
    
    2)
        echo ""
        echo "Running BEST configuration:"
        echo "  Planner: Hybrid A*"
        echo "  Controller: LQR"
        echo "  CBF: Enabled"
        echo "  Estimator: Perfect"
        echo ""
        matlab -batch "planner=3; controller=2; cbf_enabled=1; estimator=1; run('run_simulation.m');"
        ;;
    
    3)
        echo ""
        echo "Running FAST configuration:"
        echo "  Planner: Hybrid A*"
        echo "  Controller: LQR"
        echo "  CBF: Disabled"
        echo "  Estimator: Perfect"
        echo ""
        matlab -batch "planner=3; controller=2; cbf_enabled=0; estimator=1; run('run_simulation.m');"
        ;;
    
    4)
        echo ""
        echo "Running comprehensive test suite..."
        echo "This will test 6 different configurations:"
        echo "  1. Hybrid A* + LQR + CBF"
        echo "  2. Hybrid A* + MPC + CBF"
        echo "  3. Hybrid A* + LQR + CBF + EKF"
        echo "  4. RRT* + LQR + CBF"
        echo "  5. A* + LQR + CBF"
        echo "  6. Hybrid A* + LQR (no CBF)"
        echo ""
        echo "This may take 5-10 minutes..."
        matlab -batch "run('run_comprehensive_tests.m');"
        ;;
    
    5)
        echo ""
        echo "Custom Configuration"
        echo ""
        
        echo "Select planner:"
        echo "  1 = A*"
        echo "  2 = RRT*"
        echo "  3 = Hybrid A* (recommended)"
        read -p "Planner (1-3): " planner
        
        echo ""
        echo "Select controller:"
        echo "  1 = PID"
        echo "  2 = LQR (recommended)"
        echo "  3 = MPC"
        read -p "Controller (1-3): " controller
        
        echo ""
        echo "Enable CBF safety layer?"
        read -p "CBF (0=no, 1=yes): " cbf
        
        echo ""
        echo "Select state estimator:"
        echo "  1 = Perfect (ground truth)"
        echo "  2 = Noisy (Gaussian noise)"
        echo "  3 = EKF (Kalman filter)"
        echo "  4 = External (user data)"
        read -p "Estimator (1-4): " estimator
        
        echo ""
        echo "Running simulation with custom configuration..."
        matlab -batch "planner=$planner; controller=$controller; cbf_enabled=$cbf; estimator=$estimator; run('run_simulation.m');"
        ;;
    
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo ""
echo "========================================================="
echo "  Simulation Complete!"
echo "========================================================="
echo ""
echo "Check the 'matlab' directory for output figures."
echo ""
