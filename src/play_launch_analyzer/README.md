# play_launch_analyzer

Analysis and visualization tools for play_launch execution logs.

## Overview

This package provides tools to analyze and visualize resource usage metrics collected by `play_launch` during ROS 2 launch execution. It generates individual interactive HTML charts using Plotly for comprehensive performance analysis.

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

The tool generates individual interactive charts and statistics in `<log_dir>/plot/`:

**Interactive Charts** (separate HTML files, ~4-5 MB each):
- `cpu_timeline.html` - CPU usage over time
  - Containers show list of contained composable nodes on hover
- `memory_timeline.html` - Memory usage over time
  - Containers show list of contained composable nodes on hover
- `cpu_distribution.html` - CPU distribution box plot **sorted low to high by average**
  - Abbreviated labels (full names on hover)
- `memory_distribution.html` - Memory distribution box plot **sorted low to high by average**
  - Abbreviated labels (full names on hover)
- `io_timeline.html` - I/O read/write rates over time (when available)
- `network_timeline.html` - TCP/UDP connections over time (when available)
- `gpu_timeline.html` - GPU memory usage over time (when available)
- `gpu_temp_power.html` - GPU temperature over time (when available)
- `gpu_clocks.html` - GPU clock frequencies over time (when available)

**Interactive Features**:
- **Full-screen viewing**: Each chart in its own file for maximum size
- **Container-aware tooltips**: Timeline charts show which nodes run in each container
- **Abbreviated labels**: Distribution plots use short labels to save space
- **No legend clutter**: Process names appear in hover tooltips instead
- **Zoom and pan**: Drag to zoom into specific time ranges, double-click to reset
- **Hover tooltips**: Detailed values with full process names at each data point
- **Download**: Use toolbar to export charts as PNG images

**Statistics Report** (`statistics.txt`):
- Top 10 rankings for all metrics (CPU, memory, I/O, GPU, network)
- Max and average values across execution
- Comprehensive resource usage summary

**Metrics Included** (when data available):
- **CPU**: Usage percentage, user time
- **Memory**: RSS (resident set size), VMS (virtual memory size)
- **I/O**: Read/write rates (disk and total including network)
- **Network**: TCP/UDP connection counts
- **GPU** (NVIDIA only): Memory usage, compute utilization, temperature, power, clock speeds

## Requirements

- Python 3.8+
- plotly >= 5.0.0
- pandas >= 1.0.0

## License

MIT
