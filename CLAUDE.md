# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch file executions for performance analysis:
1. **dump_launch** (Python): Records launch execution to `record.json`
2. **play_launch** (Rust): Replays recorded execution with resource monitoring
3. **play_launch_wrapper** (CMake): Provides `play_launch` command in PATH
4. **play_launch_analyzer** (Python): Analyzes and visualizes logs

## Build & Usage

```sh
# Build workspace (3-stage colcon build)
make build

# Source workspace
. install/setup.bash

# Launch commands
play_launch launch <package> <launch_file>      # Record & replay
play_launch run <package> <executable>          # Run single node
play_launch dump launch <package> <launch_file> # Record only
play_launch replay --config config.yaml         # Replay with config

# Analysis
plot_play_launch                                # Plot latest logs
plot_play_launch --metrics cpu memory           # Plot specific metrics
```

## Architecture

### Execution Flow (play_launch)

1. **Load** (launch_dump.rs): Deserialize `record.json`, copy parameter files
2. **Context Preparation** (context.rs): Classify nodes (containers vs regular)
3. **Component Loader** (component_loader.rs): Background thread with rclrs service clients for LoadNode
4. **Execution** (execution.rs):
   - Regular nodes: Spawned directly
   - Composable nodes: Load via service calls to containers (or standalone with `--standalone-composable-nodes`)
5. **Container Readiness** (container_readiness.rs): Wait for LoadNode services (default: enabled)
6. **Logging**: All output saved to `play_log/<timestamp>/`

### Key Modules

- **main.rs**: Entry point, CleanupGuard for subprocess management
- **execution.rs**: Process spawning, composable node loading
- **component_loader.rs**: Direct rclrs service calls to LoadNode
- **resource_monitor.rs**: Per-node and system-wide metrics collection
- **config.rs**: YAML configuration with process control (CPU affinity, nice)
- **options.rs**: CLI parsing

## Configuration

### CLI Flags

- `--config <PATH>` (`-c`): Runtime config YAML
- `--verbose` (`-v`): Show per-node progress (default: summary only)
- `--enable-monitoring`: Enable resource monitoring
- `--monitor-interval-ms <MS>`: Sampling interval (default: 1000ms)
- `--standalone-composable-nodes`: Run composable nodes standalone
- `--load-orphan-composable-nodes`: Load nodes without matching containers
- `--disable-respawn`: Disable automatic respawn even if configured in launch file

### Config YAML Example

```yaml
composable_node_loading:
  load_node_timeout_millis: 30000
  max_concurrent_load_node_spawn: 10

container_readiness:
  wait_for_service_ready: true        # Enabled by default
  service_ready_timeout_secs: 120

monitoring:
  enabled: false
  sample_interval_ms: 1000

processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5                           # Negative nice requires CAP_SYS_NICE
```

See `test/autoware_planning_simulation/autoware_config.yaml` and `docs/resource-monitoring-design.md` for details.

## Log Directory Structure

```
play_log/YYYY-MM-DD_HH-MM-SS/
├── params_files/              # Cached parameter files
├── system_stats.csv           # System-wide metrics
├── node/<node_name>/          # Regular nodes (flat structure)
│   ├── metadata.json          # Package, namespace, container info
│   ├── metrics.csv            # Resource metrics (when enabled)
│   ├── out/err/pid/status     # Process logs
│   └── cmdline                # Executed command
└── load_node/<node_name>/     # Composable nodes (flat structure)
    ├── metadata.json
    ├── metrics.csv
    ├── service_response.*     # LoadNode responses
    └── status
```

**Features**:
- Flat 1-level structure for easy scripting
- Short directory names (e.g., `control_evaluator`) with deduplication (`_2`, `_3`)
- Each node directory is self-contained with all data

## Resource Monitoring

### Status
- ✅ **Phase 1 Complete**: Per-node and system-wide monitoring (CPU, memory, I/O, network, threads, FDs)
- ✅ **Phase 2 Complete**: GPU monitoring (NVIDIA), I/O rates, network connections, CAP_SYS_NICE
- ✅ **Phase 3.1 Complete**: Interactive Plotly dashboard with zoom/pan/hover tooltips

### Visualization
The `plot_play_launch` command generates individual interactive HTML charts using Plotly:

**Timeline Charts** (~4-5 MB each, shows metrics over time):
- `cpu_timeline.html` - CPU usage with container-aware hover panel
- `memory_timeline.html` - Memory usage with container-aware hover panel
- `io_timeline.html` - I/O read/write rates (when available)
- `network_timeline.html` - TCP/UDP connections (when available)
- `gpu_timeline.html` - GPU memory usage (when available)
- `gpu_temp_power.html` - GPU temperature and power (when available)
- `gpu_clocks.html` - GPU clock frequencies (when available)

**Distribution Charts** (box plots sorted low to high by average):
- `cpu_distribution.html` - CPU distribution with statistics panel
- `memory_distribution.html` - Memory distribution with statistics panel

**Interactive Features**:
- Full-screen viewing (separate files for maximum chart size)
- Container-aware floating panels:
  - Timeline charts: Hover over container curve → shows list of contained composable nodes
  - Distribution charts: Hover over container → shows box plot statistics + contained nodes
- Abbreviated labels in distributions, full names on hover
- No legend clutter (names in hover tooltips only)
- Zoom and pan (drag to zoom, double-click to reset)
- Download as PNG via toolbar

**Statistics Report**: `statistics.txt` with top 10 rankings for all metrics

**Implementation**: JavaScript injection pattern (inject_statistics_panel, inject_container_panel) embeds container_map as JSON, adds plotly_hover/plotly_unhover event listeners for dynamic panels

### Per-Process Metrics (metrics.csv)
CPU, memory (RSS/VMS), disk I/O, total I/O with rates, process state, threads, file descriptors, GPU (NVIDIA), TCP/UDP connections

### System-Wide Metrics (system_stats.csv)
Overall CPU%, memory, network rates, disk I/O rates, GPU stats (Jetson via jtop - pending)

### GPU Support
- **NVIDIA GPUs**: Per-process metrics via NVML (nvml-wrapper 0.11)
- **Jetson/Tegra**: ⚠️ Per-process GPU metrics NOT available (hardware limitation). Use system-wide monitoring.

### Platform Notes
- **CPU metrics**: Accurate per-process measurement via `/proc/[pid]/stat` (utime + stime). Correctly shows individual process CPU usage independent of system loading.
- **I/O metrics**: `/proc/[pid]/io` unavailable on Jetson/Tegra (warning logged once, system-wide I/O still works)
- **Negative nice values**: Requires `sudo setcap cap_sys_nice+ep install/play_launch/lib/play_launch/play_launch` (reapply after rebuild)

## Important Implementation Details

### Environment
- **User responsibility**: Source ROS setup files before running play_launch
- play_launch does NOT source setup files internally
- AMENT_PREFIX_PATH explicitly preserved for containers (node_cmdline.rs)
- Environment variables from launch files (`<env>` tags) replayed to child processes

### Process Cleanup
- **CleanupGuard**: RAII pattern kills all children on exit
- **Signal Handlers**: SIGTERM/SIGINT call `kill_all_descendants()`
- **Process Group Isolation**: `.process_group(0)` prevents children from receiving terminal signals
- Only play_launch receives Ctrl-C, then explicitly kills all descendants

### Composable Node Loading
- Service-based only (direct rclrs service calls to LoadNode)
- Service clients cached per container
- Concurrent loading (default: 10 parallel)
- Parameters auto-converted to ROS types
- Retry logic: 3 attempts, 30s timeout per node
- Namespace resolution: Relative names (e.g., `pointcloud_container`) resolved to absolute (e.g., `/pointcloud_container`)

### Respawn Support
- **Automatic Restart**: Nodes with `respawn=True` in launch files automatically restart when they exit
- **Respawn Delay**: `respawn_delay` parameter controls delay (in seconds) before restarting
- **Graceful Shutdown**: Ctrl-C immediately stops respawning nodes (fixed via tokio::sync::watch channel for persistent shutdown state)
- **Regular Nodes Only**: Currently supports respawn for regular nodes and containers (composable nodes loaded into containers do NOT auto-reload on container restart)
- **CLI Override**: Use `--disable-respawn` flag to disable all respawn behavior for testing
- **on_exit Handlers**: Not supported - warning logged if detected in launch files
- **Infinite Restarts**: Matches ROS2 launch behavior (no retry limit by default)

## Dependencies

**Rust**: tokio, rayon, eyre, clap, serde/serde_json/serde_yaml, rclrs, composition_interfaces, rcl_interfaces, sysinfo, nvml-wrapper, csv

**Python**: ruamel-yaml, pyyaml, lark, packaging, hatchling, pytest, ruff

## Testing

Autoware planning simulator integration test in `test/autoware_planning_simulation/`:
- `make start-sim`: Start simulator with play_launch
- `make drive`: Run autonomous driving test
- `make plot`: Generate resource plots
- Tested with 52 composable nodes, 15 containers

## Key Recent Fixes

- **2025-10-30**: Fixed respawn race condition - replaced `tokio::sync::Notify` with `tokio::sync::watch` channel for persistent shutdown state. Respawning nodes (like RViz) now stop immediately on Ctrl-C without spurious restarts. Watch channel provides both immediate `.borrow()` checks and awaitable `.changed()` notifications.
- **2025-10-30**: Respawn support added - nodes with respawn=True automatically restart on exit, honoring respawn_delay. Ctrl-C during respawn delay stops restart gracefully. Only regular nodes supported (composable nodes limitation documented).
- **2025-10-29**: CPU metrics completely rewritten - now parses `/proc/[pid]/stat` directly for accurate utime/stime measurement. Previous implementation incorrectly used `run_time()` (wall-clock time) instead of actual CPU time, causing all processes to show similar CPU% affected by system loading.
- **2025-10-29**: I/O warning for Jetson (`/proc/[pid]/io` unavailable)
- **2025-10-29**: Logging verbosity control (`--verbose` for per-node details)
- **2025-10-28**: NVML library loading (nvml-wrapper 0.11)
- **2025-10-27**: Namespace resolution for composable nodes
- **2025-10-26**: Flat log directory structure with metadata.json
- **2025-10-25**: AMENT_PREFIX_PATH explicit preservation
- **2025-10-22**: Service-based component loading (rclrs, no subprocess overhead)
- **2025-10-22**: Process group isolation for proper cleanup
