# ROS2 Launch Inspection Tool

A comprehensive toolkit for recording, replaying, and analyzing ROS 2 launch executions. Record any launch file once, then replay it multiple times for testing, performance analysis, and debugging.

[![Watch the demo](assets/demo.png)](assets/demo.mp4)

## Features

- **Record & Replay**: Capture launch file execution and replay it deterministically
- **Single Command Usage**: Launch and replay in one step with `play_launch launch` or `play_launch run`
- **Resource Monitoring**: Track CPU, memory, I/O, GPU usage per node
- **Visualization**: Generate comprehensive resource usage plots and statistics
- **Process Control**: Set CPU affinity and priorities for fine-grained control
- **Container Support**: Full support for ROS 2 composable nodes and containers


## Install Pre-built Packages (Recommended)

Find the latest packages in the [Release page](https://github.com/NEWSLabNTU/ros-launch-perf/releases) and follow the instructions.


## Install from Source

### Prerequisites

- **ROS 2** (Humble or later)
- **Rust toolchain**: Install from [rustup.rs](https://rustup.rs/)
- **Python 3** with pip

### Installation

1. Clone the repository:
```bash
git clone https://github.com/NEWSLabNTU/ros-launch-perf.git
cd ros-launch-perf
```

2. Install all dependencies:
```bash
make install-deps
```

This will:
- Download git submodules
- Install colcon-cargo and colcon-ros-cargo extensions
- Install cargo-ament-build for Rust ROS packages
- Install ROS dependencies via rosdep

3. Build the project:
```bash
make build
```

4. Source the workspace:
```bash
source install/setup.bash
```

5. Verify installation:
```bash
play_launch --help
```


## Getting Started

### Quick Start: Launch and Replay in One Command

The easiest way to use the tool is with the `play_launch launch` or `play_launch run` commands. These automatically record and replay in a single step:

```bash
# Launch a launch file
play_launch launch <package_name> <launch_file> [arguments...]

# Example: Launch a demo
play_launch launch demo_nodes_cpp talker_listener.launch.py

# Example: Run a single node
play_launch run demo_nodes_cpp talker
```

This is functionally equivalent to `ros2 launch` or `ros2 run`, but records the execution and replays it immediately.

### Advanced: Two-Step Workflow (Dump + Replay)

For more control, you can separate the recording and replay steps:

#### Step 1: Record (Dump) a Launch Execution

Record any ROS 2 launch file by replacing `ros2 launch` with `play_launch dump launch`:

```bash
# Original command:
# ros2 launch autoware_launch planning_simulator.launch.xml \
#     map_path:=$HOME/autoware_map/sample-map-planning \
#     vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# Recording command:
play_launch dump launch \
    autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

This creates a `record.json` file capturing the entire launch execution plan.

#### Step 2: Replay the Launch

Replay the recorded launch multiple times:

```bash
play_launch replay
```

The replay reads `record.json` and executes the launch plan, spawning all nodes and containers exactly as recorded.


## Resource Monitoring

Enable comprehensive per-node resource monitoring to track CPU, memory, I/O, and GPU usage.

### Enable Monitoring

Monitor all nodes with default settings (1 second interval):

```bash
play_launch launch demo_nodes_cpp talker_listener.launch.py --enable-monitoring
```

Or when replaying:

```bash
play_launch replay --enable-monitoring
```

### Configure Monitoring

For fine-grained control, use a YAML configuration file:

```yaml
# config.yaml
monitoring:
  enabled: true
  sample_interval_ms: 1000  # Sample every 1 second

processes:
  # Monitor all containers with CPU affinity
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5

  # Specific node with higher priority
  - node_pattern: "NODE 'rviz2/rviz2*"
    monitor: true
    nice: -5  # Requires CAP_SYS_NICE capability
```

Apply the configuration:

```bash
play_launch replay --config config.yaml
```

### Monitoring Output

Metrics are saved to `play_log/<timestamp>/node/<node_name>/metrics.csv` and `play_log/<timestamp>/load_node/<node_name>/metrics.csv` with the following data:

- **CPU**: Usage percentage, user/system time
- **Memory**: RSS, VMS
- **I/O**: Read/write bytes, rates
- **Network**: TCP/UDP connection counts
- **GPU** (if available): Memory, utilization, temperature, power
- **Process**: State, thread count, file descriptor count

> **Note on GPU Monitoring**: Per-process GPU metrics are NOT available on Jetson/Tegra platforms (AGX Orin, Xavier, Nano) due to hardware architecture limitations. GPU columns will be empty on these systems. CPU, memory, and I/O metrics continue to work normally. See [CLAUDE.md](CLAUDE.md) for details and Jetson-specific GPU monitoring alternatives.


## Visualization and Analysis

Generate comprehensive interactive plots and statistics from monitoring data:

```bash
# Plot latest execution
plot_play_launch

# Plot specific log directory
plot_play_launch --log-dir play_log/2025-10-28_16-17-56

# Plot only CPU and memory
plot_play_launch --metrics cpu memory

# List available metrics
plot_play_launch --list-metrics
```

### Generated Output

Interactive charts are saved to `play_log/<timestamp>/plot/` as separate HTML files (~4-5 MB each):

**Timeline Charts** (show metrics over time):
- `cpu_timeline.html` - CPU usage over time
- `memory_timeline.html` - Memory usage over time
- `io_timeline.html` - I/O read/write rates (when available)
- `network_timeline.html` - TCP/UDP connections (when available)
- `gpu_timeline.html` - GPU memory usage (when available)
- `gpu_temp_power.html` - GPU temperature and power (when available)
- `gpu_clocks.html` - GPU clock frequencies (when available)

**Distribution Charts** (statistical distributions):
- `cpu_distribution.html` - CPU distribution box plot sorted by average
- `memory_distribution.html` - Memory distribution box plot sorted by average

**Statistics Report**:
- `statistics.txt` - Top 10 rankings for all metrics (max/avg)

### Interactive Features

- **Full-screen viewing**: Each chart in its own file for maximum size
- **Container awareness**:
  - Timeline charts: Hover over a container curve to see list of contained nodes in a floating panel
  - Distribution charts: Hover over a container to see statistics + contained nodes in floating panel
- **Abbreviated labels**: Distribution plots use short labels to save space, full names shown on hover
- **No legend clutter**: Process names appear in hover tooltips instead of a legend
- **Zoom and pan**: Drag to zoom into specific time ranges, double-click to reset
- **Hover tooltips**: Detailed values with full process names at each data point
- **Download**: Use toolbar to export charts as PNG images

### Statistics Report

Example statistics in `statistics.txt`:
- Top 10 nodes by CPU/memory usage (max and average)
- Top 10 nodes by I/O rates
- Top 10 nodes by GPU utilization (when available)
- Top 10 nodes by network connections


## Command Reference

```bash
# All-in-one launch and replay
play_launch launch <package> <launch_file> [args...]
play_launch run <package> <executable> [args...]

# Separate dump and replay
play_launch dump launch <package> <launch_file> [args...]
play_launch replay [--input-file record.json]

# With monitoring
play_launch launch <package> <launch_file> --enable-monitoring
play_launch replay --enable-monitoring --monitor-interval-ms 500

# With configuration
play_launch replay --config config.yaml

# Verbose output
play_launch replay --verbose

# Plot results
plot_play_launch [--log-dir <dir>] [--metrics cpu memory io gpu]
```


## Development

### Linting

Lint both Python and Rust code:

```sh
make lint
```

### Formatting

Format both Python and Rust code:

```sh
make format
```

### Testing

Run the full test suite (both Python and Rust):

```sh
make test
```


## License

This software is distributed under MIT license. You can read the [license file](LICENSE.txt).
