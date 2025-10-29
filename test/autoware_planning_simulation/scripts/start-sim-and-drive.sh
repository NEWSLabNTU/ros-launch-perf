#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
cd "$SCRIPT_DIR"

MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"

source ../../install/setup.bash
source autoware/install/setup.bash
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"

# Define the two commands as functions
start_simulator() {
    play_launch launch \
        autoware_launch planning_simulator.launch.xml \
        map_path:="$MAP_PATH"
}

run_test() {
    echo "[Test] Waiting 60s for system initialization..."
    sleep 60

    echo "[Test] Running autonomous driving test..."
    python3 scripts/test_autonomous_drive.py
    TEST_STATUS=$?

    if [ $TEST_STATUS -eq 0 ]; then
        echo "[Test] Test completed successfully"
        echo "[Test] System will continue running. Press Ctrl+C to stop."
    else
        echo "[Test] Test failed with exit code $TEST_STATUS"
        # Kill the parallel job group
        kill 0
        exit $TEST_STATUS
    fi
}

# Export functions and variables for parallel
export -f start_simulator run_test

# Run both tasks in parallel using GNU Parallel
# --line-buffer: Print output line-by-line (not mixed)
# --halt now,fail=1: Stop all jobs immediately if one fails
# --jobs 2: Run 2 jobs in parallel
# --keep-order: Maintain order of output
parallel --line-buffer --halt now,fail=1 --jobs 2 ::: start_simulator run_test

echo ""
echo "Full test sequence completed"
