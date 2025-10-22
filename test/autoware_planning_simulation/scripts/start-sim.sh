#!/bin/bash
set -e

# Start Autoware planning simulator with play_launch
# This script is extracted from the Makefile 'run' target

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
echo "Starting Autoware with play_launch (monitoring enabled)..."

# Determine ros-launch-perf workspace path
ROS_LAUNCH_PERF_WS="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_LAUNCH_PERF_SETUP="$ROS_LAUNCH_PERF_WS/install/setup.bash"

# Source Autoware setup first
cd "$AUTOWARE_PATH/install"
source setup.bash 2>&1 >/dev/null

# Then source ros-launch-perf workspace (prepends to AMENT_PREFIX_PATH)
if [ -f "$ROS_LAUNCH_PERF_SETUP" ]; then
    source "$ROS_LAUNCH_PERF_SETUP" 2>&1 >/dev/null
else
    echo "ERROR: ros-launch-perf setup.bash not found at $ROS_LAUNCH_PERF_SETUP"
    echo "Please build the workspace with: cd $ROS_LAUNCH_PERF_WS && make build"
    exit 1
fi

# Return to script directory
cd "$SCRIPT_DIR"

# Set CycloneDDS configuration
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"

echo "Using CycloneDDS configuration: $CYCLONEDDS_URI"
echo "Verifying configuration file exists..."
if [ ! -f "$SCRIPT_DIR/cyclonedds.xml" ]; then
    echo "ERROR: CycloneDDS configuration file not found at $SCRIPT_DIR/cyclonedds.xml"
    exit 1
fi

# Dump launch execution
ros2 run dump_launch dump_launch \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$MAP_PATH"

# Replay with play_launch
echo "Starting play_launch with CYCLONEDDS_URI=$CYCLONEDDS_URI"
ros2 run play_launch play_launch \
    --enable-monitoring \
    --monitor-interval-ms 1000 \
    --wait-for-service-ready \
    --service-ready-timeout-secs "$SERVICE_TIMEOUT" \
    --load-node-timeout-millis "$LOAD_TIMEOUT"
