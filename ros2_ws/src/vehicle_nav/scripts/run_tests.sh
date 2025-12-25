#!/bin/bash
# Automated testing script for all planner/controller combinations

# Test configurations
PLANNERS=("astar" "rrtstar")
CONTROLLERS=("pid" "lqr" "mpc")
CBF_OPTIONS=("true" "false")
GOAL_X=8.0
GOAL_Y=8.0
TIMEOUT=60  # seconds per test
RESULTS_DIR="test_results"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  ROS2 Navigation System Test Suite"
echo "=========================================="
echo ""

# Create results directory
mkdir -p $RESULTS_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY_FILE="$RESULTS_DIR/summary_${TIMESTAMP}.txt"

# Initialize summary
echo "Test Results - $(date)" > $SUMMARY_FILE
echo "========================================" >> $SUMMARY_FILE
echo "" >> $SUMMARY_FILE

# Source ROS2 workspace
source install/setup.bash

# Test counter
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Run all combinations
for planner in "${PLANNERS[@]}"; do
    for controller in "${CONTROLLERS[@]}"; do
        for use_cbf in "${CBF_OPTIONS[@]}"; do
            
            TOTAL_TESTS=$((TOTAL_TESTS + 1))
            
            # Test name
            if [ "$use_cbf" == "true" ]; then
                TEST_NAME="${planner}_${controller}_cbf"
            else
                TEST_NAME="${planner}_${controller}_nocbf"
            fi
            
            OUTPUT_FILE="$RESULTS_DIR/${TEST_NAME}_${TIMESTAMP}.json"
            
            echo ""
            echo -e "${YELLOW}[Test $TOTAL_TESTS] Running: $TEST_NAME${NC}"
            echo "  Planner: $planner"
            echo "  Controller: $controller"
            echo "  CBF: $use_cbf"
            echo ""
            
            # Launch system in background
            ros2 launch vehicle_nav sim_launch.py \
                planner:=$planner \
                controller:=$controller \
                use_cbf:=$use_cbf \
                use_rviz:=false \
                > /dev/null 2>&1 &
            LAUNCH_PID=$!
            
            # Wait for system to initialize
            sleep 3
            
            # Start metrics collector in background
            ros2 run vehicle_nav metrics_node \
                --ros-args \
                -p test_name:=$TEST_NAME \
                -p timeout:=$TIMEOUT \
                -p output_file:=$OUTPUT_FILE \
                > /dev/null 2>&1 &
            METRICS_PID=$!
            
            # Wait a bit more
            sleep 2
            
            # Publish goal
            echo "  Publishing goal: ($GOAL_X, $GOAL_Y)"
            ros2 run vehicle_nav goal_publisher $GOAL_X $GOAL_Y > /dev/null 2>&1
            
            # Wait for test to complete (timeout + buffer)
            sleep $((TIMEOUT + 5))
            
            # Kill processes
            kill $LAUNCH_PID 2>/dev/null
            kill $METRICS_PID 2>/dev/null
            pkill -f "ros2 launch vehicle_nav" 2>/dev/null
            pkill -f "ros2 run vehicle_nav" 2>/dev/null
            
            # Wait for processes to die
            sleep 2
            
            # Check if results file exists and parse it
            if [ -f "$OUTPUT_FILE" ]; then
                GOAL_REACHED=$(grep -o '"goal_reached": [a-z]*' "$OUTPUT_FILE" | awk '{print $2}')
                EXEC_TIME=$(grep -o '"execution_time": [0-9.]*' "$OUTPUT_FILE" | awk '{print $2}')
                TRACKING_ERROR=$(grep -o '"mean_tracking_error": [0-9.]*' "$OUTPUT_FILE" | awk '{print $2}')
                SAFETY_VIOLS=$(grep -o '"safety_violations": [0-9]*' "$OUTPUT_FILE" | awk '{print $2}')
                
                if [ "$GOAL_REACHED" == "true" ]; then
                    echo -e "${GREEN}  ✓ PASSED${NC} - Goal reached in ${EXEC_TIME}s"
                    echo "    Tracking error: ${TRACKING_ERROR}m, Safety violations: ${SAFETY_VIOLS}"
                    PASSED_TESTS=$((PASSED_TESTS + 1))
                    
                    # Add to summary
                    echo "✓ $TEST_NAME: PASSED (${EXEC_TIME}s, error: ${TRACKING_ERROR}m)" >> $SUMMARY_FILE
                else
                    echo -e "${RED}  ✗ FAILED${NC} - Goal not reached"
                    FAILED_TESTS=$((FAILED_TESTS + 1))
                    
                    # Add to summary
                    echo "✗ $TEST_NAME: FAILED (timeout)" >> $SUMMARY_FILE
                fi
            else
                echo -e "${RED}  ✗ FAILED${NC} - No results file generated"
                FAILED_TESTS=$((FAILED_TESTS + 1))
                
                # Add to summary
                echo "✗ $TEST_NAME: FAILED (no metrics)" >> $SUMMARY_FILE
            fi
            
            # Clean up any remaining processes
            pkill -9 -f "ros2" 2>/dev/null
            sleep 1
            
        done
    done
done

# Print final summary
echo ""
echo "=========================================="
echo "  Test Summary"
echo "=========================================="
echo "Total Tests: $TOTAL_TESTS"
echo -e "Passed: ${GREEN}$PASSED_TESTS${NC}"
echo -e "Failed: ${RED}$FAILED_TESTS${NC}"
echo "Success Rate: $((PASSED_TESTS * 100 / TOTAL_TESTS))%"
echo ""
echo "Results saved to: $RESULTS_DIR"
echo "Summary: $SUMMARY_FILE"
echo "=========================================="

# Add summary to file
echo "" >> $SUMMARY_FILE
echo "========================================" >> $SUMMARY_FILE
echo "Total: $TOTAL_TESTS | Passed: $PASSED_TESTS | Failed: $FAILED_TESTS" >> $SUMMARY_FILE
echo "Success Rate: $((PASSED_TESTS * 100 / TOTAL_TESTS))%" >> $SUMMARY_FILE
