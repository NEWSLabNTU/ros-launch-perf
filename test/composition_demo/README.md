# ROS Composition Demo Test

This directory tests the service-based component loading using ROS official composition examples.

## Test Setup

This test uses the official ROS `composition` package demos to verify:
1. Recording of launch files with composable nodes
2. Replaying with service-based component loading
3. Verifying nodes load correctly into containers

## Test Cases

### 1. Basic Composition Demo
- Package: `composition`
- Launch file: `composition_demo.launch.py`
- Components: Talker + Listener in a single container
- Container: `my_container`

## Requirements

- ROS 2 Humble
- `composition` package (installed with ros-humble-desktop)
- Sourced workspace: `. /opt/ros/humble/setup.bash && . install/setup.bash`

## Quick Start

```bash
# Run all tests
make test-all

# Individual tests
make test-record       # Record the composition demo
make test-replay       # Replay using play_launch
make test-replay-standalone  # Replay in standalone mode
make test-verify       # Verify the replay worked

# Clean up
make clean
```

## Expected Results

After successful replay:
- Container process spawned
- Talker and Listener loaded via LoadNode service calls
- Output log shows successful loading
- No orphan processes remain

## Debugging

```bash
# Check the recorded launch dump
cat record.json | jq .

# View logs
ls -la play_log/

# Check specific node logs
cat play_log/*/node/rclcpp_components/component_container/out
cat play_log/*/load_node/my_container/composition/composition::Talker/service_response.0
```

## Comparison with CLI

The old approach would use:
```bash
ros2 component load /my_container composition composition::Talker
```

The new approach uses direct rclrs service calls to the same LoadNode service.
