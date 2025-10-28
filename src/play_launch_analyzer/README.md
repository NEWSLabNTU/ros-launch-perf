# play_launch_analyzer

Analysis and visualization tools for play_launch execution logs.

## Overview

This package provides tools to analyze and visualize resource usage metrics collected by `play_launch` during ROS 2 launch execution.

## Installation

Build the workspace:

```bash
cd /path/to/ros-launch-perf
make build
source install/setup.bash
```

## Usage

### plot_play_launch

Generate resource usage plots from play_launch logs:

```bash
# Plot all metrics from latest log in ./play_log
plot_play_launch

# List available metrics in a log
plot_play_launch --log-dir play_log/2025-10-28_16-17-56 --list-metrics

# Plot specific metrics only
plot_play_launch --metrics cpu memory
plot_play_launch --metrics io
plot_play_launch --metrics gpu

# Plot from specific log directory (absolute or relative)
plot_play_launch --log-dir /path/to/play_log/2025-10-28_16-17-56
plot_play_launch --log-dir ../other/play_log/2025-10-28_16-17-56

# Specify custom base log directory
plot_play_launch --base-log-dir /path/to/logs

# Specify output directory for plots
plot_play_launch --output-dir /path/to/output

# Combine options
plot_play_launch --log-dir ./play_log/latest --metrics cpu memory io --output-dir ./analysis
```

### Command Options

- `--log-dir PATH`: Specific log directory (absolute or relative path)
- `--base-log-dir PATH`: Base directory containing timestamped logs (default: ./play_log)
- `--output-dir PATH`: Output directory for plots (default: <log_dir>/plots)
- `--metrics {cpu,memory,io,gpu,all}`: Select which metrics to plot (default: all)
- `--list-metrics`: List available metrics in the log and exit
- `-h, --help`: Show help message

### Generated Outputs

The tool generates comprehensive plots in `<log_dir>/plots/`:

**CPU and Memory:**
- `cpu_usage.png` - CPU usage timeline
- `memory_usage.png` - Memory usage timeline
- `cpu_distribution.png` - CPU distribution box plot
- `memory_distribution.png` - Memory distribution box plot

**I/O (when available):**
- `io_read_usage.png` - I/O read rate timeline
- `io_write_usage.png` - I/O write rate timeline
- `io_distribution.png` - I/O rate distribution

**GPU (when available):**
- `gpu_memory_usage.png` - GPU memory usage timeline
- `gpu_utilization.png` - GPU compute utilization timeline
- `gpu_temperature.png` - GPU temperature timeline
- `gpu_power.png` - GPU power consumption timeline
- `gpu_clocks.png` - GPU graphics and memory clocks timeline
- `gpu_distribution.png` - GPU metrics distribution

**Summary:**
- `legend.png` - Node index legend mapping
- `statistics.txt` - Comprehensive statistics report
- `containers.txt` - Container listing

## Requirements

- Python 3.8+
- matplotlib
- numpy

## License

MIT
