#!/bin/bash
set -e

# Start Autoware planning simulator with standard ros2 launch (for comparison with play_launch)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
AUTOWARE_LINK="$SCRIPT_DIR/autoware"
AUTOWARE_PATH="$(readlink -f "$AUTOWARE_LINK" 2>/dev/null)"
AUTOWARE_SETUP="$AUTOWARE_PATH/install/setup.bash"

# Default configuration
MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"

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
echo "Starting Autoware with standard ros2 launch (for comparison)..."

# Source Autoware setup
cd "$AUTOWARE_PATH"
source install/setup.bash 2>&1 >/dev/null

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

# Launch with standard ros2 launch
echo "Starting Autoware with ros2 launch..."
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:="$MAP_PATH"
