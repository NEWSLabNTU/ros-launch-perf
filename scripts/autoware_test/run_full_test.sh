#!/usr/bin/env bash
# Full automated test: start Autoware with play_launch, then run autonomous driving test

set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Cleanup function
cleanup() {
    log_warn "Cleaning up..."

    # Kill play_launch if running
    pkill -f "play_launch" 2>/dev/null || true

    # Kill orphan nodes
    if [ -f "../../scripts/kill_orphan_nodes.sh" ]; then
        bash ../../scripts/kill_orphan_nodes.sh
    fi
}

# Set trap for cleanup on exit
trap cleanup EXIT INT TERM

log_info "=========================================="
log_info "Full Autoware Autonomous Driving Test"
log_info "=========================================="

# Step 1: Start Autoware with play_launch
log_info ""
log_info "Step 1: Starting Autoware with play_launch..."
log_info "This will run in the background. Logs will be saved to play_log/"

# Run the launch script in the background
bash ./run.sh &
PLAY_LAUNCH_PID=$!

log_info "play_launch started (PID: $PLAY_LAUNCH_PID)"

# Step 2: Wait for Autoware to be ready
log_info ""
log_info "Step 2: Waiting for Autoware to initialize..."
log_info "This typically takes 30-60 seconds..."

# Wait progressively
sleep 10
log_info "  [10s] System starting..."

sleep 10
log_info "  [20s] Loading components..."

sleep 10
log_info "  [30s] Initializing services..."

sleep 10
log_info "  [40s] Almost ready..."

sleep 10
log_info "  [50s] Performing final checks..."

sleep 10
log_info "  [60s] System should be ready now"

# Step 3: Run the autonomous driving test
log_info ""
log_info "Step 3: Running autonomous driving test..."
log_info "This will set initial pose, goal, and engage autonomous mode"

python3 ./test_autonomous_drive.py

TEST_STATUS=$?

# Step 4: Keep running for observation
if [ $TEST_STATUS -eq 0 ]; then
    log_info ""
    log_info "=========================================="
    log_info "Test sequence completed successfully!"
    log_info "=========================================="
    log_info ""
    log_info "The vehicle is now driving autonomously."
    log_info "You can:"
    log_info "  - Monitor progress in RViz (DISPLAY=:2)"
    log_info "  - Check logs in play_log/"
    log_info "  - Press Ctrl+C to stop"
    log_info ""
    log_info "Keeping system running for 5 minutes..."

    # Keep running and monitoring
    for i in {1..60}; do
        sleep 5
        echo -n "."
    done

    log_info ""
    log_info "5 minutes elapsed. Test complete."
else
    log_error ""
    log_error "Test sequence failed with exit code $TEST_STATUS"
    log_error "Check the output above for errors"
    log_error ""
fi

# Cleanup will be called automatically by trap

exit $TEST_STATUS
