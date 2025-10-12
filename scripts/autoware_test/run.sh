#!/usr/bin/env bash
set -e

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

# Check if autoware symlink exists and points to a valid directory
if [ ! -L "autoware" ]; then
    echo "ERROR: 'autoware' symlink not found in $script_dir"
    echo "Please create a symlink pointing to your Autoware installation:"
    echo "  ln -s /path/to/autoware autoware"
    exit 1
fi

autoware_path=$(readlink "autoware")
if [ -z "$autoware_path" ]; then
    echo "ERROR: Failed to read 'autoware' symlink"
    exit 1
fi

# Resolve to absolute path if relative
if [[ "$autoware_path" != /* ]]; then
    autoware_path="$script_dir/$autoware_path"
fi

if [ ! -d "$autoware_path" ]; then
    echo "ERROR: 'autoware' symlink points to non-existent directory: $autoware_path"
    echo "Current symlink target: $(readlink autoware)"
    echo "Please update the symlink to point to a valid Autoware installation:"
    echo "  ln -sf /path/to/autoware autoware"
    exit 1
fi

if [ ! -f "$autoware_path/install/setup.bash" ]; then
    echo "ERROR: Autoware installation at $autoware_path appears to be incomplete"
    echo "Could not find install/setup.bash"
    echo "Please ensure the symlink points to a built Autoware workspace"
    exit 1
fi

echo "Using Autoware installation at: $autoware_path"

source autoware/install/setup.bash
dump_launch \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$HOME/autoware_map/sample-map-planning" \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
play_launch \
    --wait-for-service-ready \
    --service-ready-timeout-secs 300 \
    --load-node-timeout-millis 60000

