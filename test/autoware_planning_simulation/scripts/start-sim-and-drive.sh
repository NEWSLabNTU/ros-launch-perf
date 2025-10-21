#!/bin/bash
set -e

# Run full test sequence: start simulator and autonomous driving test in parallel
# Uses GNU Parallel to manage both tasks

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
AUTOWARE_LINK="$SCRIPT_DIR/autoware"
AUTOWARE_PATH="$(readlink -f "$AUTOWARE_LINK" 2>/dev/null)"
AUTOWARE_SETUP="$AUTOWARE_PATH/install/setup.bash"

# Default configuration
MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"
SERVICE_TIMEOUT="${SERVICE_TIMEOUT:-300}"
LOAD_TIMEOUT="${LOAD_TIMEOUT:-60000}"

# Check Autoware installation
if [ ! -L "$AUTOWARE_LINK" ]; then
    echo "ERROR: 'autoware' symlink not found in $SCRIPT_DIR"
    echo "Create symlink: ln -s /path/to/autoware $SCRIPT_DIR/autoware"
    exit 1
fi

if [ -z "$AUTOWARE_PATH" ] || [ ! -d "$AUTOWARE_PATH" ]; then
    echo "ERROR: autoware symlink points to invalid path"
    exit 1
fi

if [ ! -f "$AUTOWARE_SETUP" ]; then
    echo "ERROR: install/setup.bash not found at $AUTOWARE_SETUP"
    exit 1
fi

echo "Autoware installation: $AUTOWARE_PATH"
echo "Starting full test sequence..."
echo ""

# Define the two commands as functions
start_simulator() {
    echo "[Simulator] Starting Autoware planning simulator..."
    cd "$AUTOWARE_PATH/install"
    source setup.bash 2>&1 >/dev/null
    cd "$SCRIPT_DIR"

    dump_launch \
        autoware_launch planning_simulator.launch.xml \
        map_path:="$MAP_PATH"

    play_launch \
        --wait-for-service-ready \
        --service-ready-timeout-secs "$SERVICE_TIMEOUT" \
        --load-node-timeout-millis "$LOAD_TIMEOUT"
}

run_test() {
    echo "[Test] Waiting 60s for system initialization..."
    sleep 60

    echo "[Test] Running autonomous driving test..."
    cd "$AUTOWARE_PATH/install"
    source setup.bash 2>&1 >/dev/null
    cd "$SCRIPT_DIR"

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
export AUTOWARE_PATH SCRIPT_DIR MAP_PATH SERVICE_TIMEOUT LOAD_TIMEOUT

# Run both tasks in parallel using GNU Parallel
# --line-buffer: Print output line-by-line (not mixed)
# --halt now,fail=1: Stop all jobs immediately if one fails
# --jobs 2: Run 2 jobs in parallel
# --keep-order: Maintain order of output
parallel --line-buffer --halt now,fail=1 --jobs 2 ::: start_simulator run_test

echo ""
echo "Full test sequence completed"
