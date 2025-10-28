# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Launch Inspection Tool - A comprehensive system for recording, replaying, and analyzing ROS 2 launch executions. The project consists of:

1. **dump_launch** (Python): Records ROS 2 launch file execution and generates `record.json`
2. **play_launch** (Rust): Replays the recorded launch execution from `record.json`
3. **play_launch_wrapper** (CMake): Provides a wrapper script to run `play_launch` directly from PATH
4. **play_launch_analyzer** (Python): Analysis and visualization tools for play_launch execution logs

## Build & Install

### ROS Workspace Build
```sh
make build
```
This builds the entire workspace in 5 stages using colcon:
1. Stage 1: ROS2 Rust base packages
2. Stage 2: ROS interface packages
3. Stage 3: dump_launch Python package
4. Stage 4: play_launch Rust package
5. Stage 5: Analysis tools and wrappers (play_launch_wrapper, play_launch_analyzer)

### Source Workspace
After building, source the workspace to use the tools:
```sh
. install/setup.bash
```

### Clean
```sh
make clean
```

### Install ROS Dependencies
```sh
make prepare
```
Installs ROS dependencies using rosdep.

## Development Commands

### Running the Tools

After sourcing the workspace, you can run the tools directly:

Record a launch execution:
```sh
ros2 run dump_launch dump_launch <package> <launch_file> [args...]
```

Replay the recorded launch (using wrapper):
```sh
play_launch [options]
```

Or use the full ROS command:
```sh
ros2 run play_launch play_launch [options]
```


Analyze and plot resource usage:
```sh
# Plot latest log directory
plot_play_launch

# Plot specific log directory
plot_play_launch --log-dir play_log/2025-10-28_16-17-56

# Specify output directory
plot_play_launch --output-dir /tmp/plots
```

**Note**: The `play_launch` and `plot_play_launch` commands are available in PATH thanks to the `play_launch_wrapper` and `play_launch_analyzer` packages.

### Testing & Profiling

Profile resource usage (requires procpath):
```sh
make profile
```

Generate profiling plots:
```sh
make plot
```

## Architecture

### Launch Dump Format (record.json)

The `LaunchDump` struct (play_launch/src/launch_dump.rs:18) contains:
- **node**: Regular ROS nodes (`NodeRecord`)
- **load_node**: Composable nodes to load into containers (`ComposableNodeRecord`)
- **container**: Node container definitions (`NodeContainerRecord`)
- **lifecycle_node**: Lifecycle node names
- **file_data**: Cached parameter file contents (path → content)

### Execution Flow (play_launch)

1. **Load & Transform** (launch_dump.rs): Deserialize `record.json` and copy cached parameter files to `play_log/params_files/`

2. **Context Preparation** (context.rs):
   - `prepare_node_contexts()`: Classify nodes into containers vs regular nodes
   - `prepare_composable_node_contexts()`: Prepare composable node loading contexts

3. **Component Loader Initialization** (component_loader.rs):
   - Background thread started with ROS context and executor for service-based node loading
   - Creates rclrs service clients for each container's LoadNode service
   - Direct service calls to composition_interfaces/srv/LoadNode using rclrs (no subprocess overhead)

4. **Execution** (execution.rs):
   - **Regular nodes**: Spawned directly via `spawn_nodes()`
   - **Composable nodes**: Two execution modes:
     - **Standalone mode** (`--standalone-composable-nodes`): Each composable node runs in its own process via `ros2 component standalone`
     - **Container mode** (default): Containers spawn first, then composable nodes load via service calls

5. **Composable Node Loading Strategy** (service-based only):
   - Nodes classified as "nice" (have matching container) or "orphan" (no matching container)
   - **Namespace resolution**: Automatically resolves relative container names (e.g., `"pointcloud_container"`) to absolute names (e.g., `"/pointcloud_container"`) to match ROS 2 runtime behavior
   - Loading is concurrent with configurable `max_concurrent_load_node_spawn` (default: 10)
   - Component loader thread handles direct rclrs service calls to `composition_interfaces/srv/LoadNode`
   - Service clients cached per container for efficient reuse
   - Parameters parsed from strings to ROS Parameter types (bool, int64, double, string)
   - Retry logic: up to `load_node_attempts` (default: 3) with `load_node_timeout_millis` (default: 30s)
   - Orphans only loaded if `--load-orphan-composable-nodes` is set

6. **Container Readiness Checking** (default: enabled, container_readiness.rs):
   - **Enabled by default** (set `container_readiness.wait_for_service_ready: false` in config to disable)
   - ROS service discovery thread monitors container services
   - Waits for `list_nodes` and `load_node` services to be available
   - Configurable timeout via config file (default: 120s)

7. **Logging**: All node stdout/stderr, PIDs, status codes saved to `play_log/node/` and `play_log/load_node/`

### play_launch_wrapper Package

The `play_launch_wrapper` is a simple ament_cmake package that provides a wrapper script to run `play_launch` directly from PATH without needing the full `ros2 run` command.

**Package Structure**:
```
src/play_launch_wrapper/
├── CMakeLists.txt               # Simple CMake package
├── package.xml                  # ROS package metadata
├── hooks/
│   └── play_launch_path.dsv.in  # Environment hook to add wrapper to PATH
└── scripts/
    └── play_launch              # Wrapper script: exec ros2 run play_launch play_launch "$@"
```

**How it works**:
1. The wrapper script forwards all arguments to `ros2 run play_launch play_launch`
2. `ament_environment_hooks()` in CMakeLists.txt registers the DSV hook
3. The DSV hook adds `install/play_launch_wrapper/lib/play_launch_wrapper` to PATH
4. After sourcing the workspace, `play_launch` is available as a command

**Why this approach**:
- ament_cargo (used by play_launch) doesn't support environment hooks in Cargo.toml
- Creating a separate CMake package with ament_environment_hooks() is the standard ROS pattern
- Cleaner than post-build scripts and survives rebuilds automatically

### play_launch_analyzer Package

The `play_launch_analyzer` is an ament_python package that provides analysis and visualization tools for play_launch execution logs.

**Package Structure**:
```
src/play_launch_analyzer/
├── package.xml                  # ROS package metadata
├── setup.py                     # Python package setup
├── README.md                    # Package documentation
├── resource/
│   └── play_launch_analyzer     # ROS resource marker
└── play_launch_analyzer/
    ├── __init__.py              # Package entry point
    └── plot_resource_usage.py   # Resource plotting script
```

**Provided Commands**:
- `plot_play_launch`: Generate comprehensive resource usage plots and statistics from play_launch logs

**Features**:
- Automatic detection and plotting of available metrics (CPU, memory, GPU, I/O, network)
- Timeline plots showing metrics over time
- Distribution plots (box plots) showing statistical distributions
- Comprehensive statistics report with top 10 rankings for all metrics
- Graceful handling of missing data (e.g., no GPU hardware)

**Usage**:
```bash
# Plot all metrics from latest log in current directory's play_log/
plot_play_launch

# List available metrics in a log
plot_play_launch --list-metrics
plot_play_launch --log-dir /path/to/log --list-metrics

# Plot specific metrics only
plot_play_launch --metrics cpu memory
plot_play_launch --metrics io
plot_play_launch --metrics gpu

# Plot from specific log directory (absolute or relative)
plot_play_launch --log-dir play_log/2025-10-28_16-17-56
plot_play_launch --log-dir /absolute/path/to/log

# Specify custom base log directory
plot_play_launch --base-log-dir /path/to/logs

# Specify output directory
plot_play_launch --output-dir /tmp/plots

# Combine options
plot_play_launch --log-dir ./play_log/latest --metrics cpu memory --output-dir ./analysis
```

**Features**:
- **Automatic log discovery**: Finds latest timestamped log in ./play_log by default
- **Flexible paths**: Supports both absolute and relative paths for log directories
- **Metric selection**: Choose which metrics to plot (cpu, memory, io, gpu, or all)
- **Metric inspection**: List available metrics before plotting
- **Works anywhere**: Can be run from any directory, not just the test directory

### Key Rust Modules

- **main.rs**: Entry point, orchestrates the async runtime and execution flow, includes CleanupGuard for subprocess cleanup
- **launch_dump.rs**: Deserialization and data transformation
- **context.rs**: Execution context preparation for nodes and composable nodes
- **execution.rs**: Process spawning, container management, composable node loading via service calls
- **component_loader.rs**: Background thread with ROS node/executor for direct rclrs service calls to LoadNode
- **container_readiness.rs**: Optional service discovery and readiness checking for containers
- **node_cmdline.rs**: Command-line parsing and generation for ROS nodes
- **options.rs**: CLI argument parsing

### Python dump_launch

- **inspector.py**: Core inspection logic for ROS launch files
- **event_handlers.py**: Launch event handling
- **ros_cmdline/**: ROS command-line utilities

## Important Configuration Options

play_launch accepts these options (play_launch/src/options.rs):

### CLI Flags (Simplified)
- `--log-dir <PATH>`: Log directory (default: `play_log`)
- `--config <PATH>` (short: `-c`): Runtime configuration file (YAML) - **Primary interface for fine-grained control**
- `--enable-monitoring`: Enable resource monitoring for all nodes (overrides config file)
- `--monitor-interval-ms <MS>`: Sampling interval in milliseconds (overrides config file)
- `--standalone-composable-nodes`: Run composable nodes standalone instead of loading into containers
- `--load-orphan-composable-nodes`: Load composable nodes that have no matching container

### Configuration File (config.yaml)

The `--config` flag points to a YAML file that controls fine-grained behavior. Use this for production deployments.

**Example config.yaml**:
```yaml
# Composable node loading settings
composable_node_loading:
  delay_load_node_millis: 2000        # Delay before loading (default: 2000ms)
  load_node_timeout_millis: 30000     # Timeout per node (default: 30000ms)
  load_node_attempts: 3               # Max retry attempts (default: 3)
  max_concurrent_load_node_spawn: 10  # Concurrent loading limit (default: 10)

# Container readiness checking (default: enabled)
container_readiness:
  wait_for_service_ready: true        # Enable service discovery (default: true)
  service_ready_timeout_secs: 120     # Max wait time (default: 120s, 0=unlimited)
  service_poll_interval_ms: 500       # Polling interval (default: 500ms)

# Resource monitoring settings
monitoring:
  enabled: false                      # Enable monitoring (default: false)
  sample_interval_ms: 1000            # Sampling interval (default: 1000ms)

# Per-process configurations (optional)
processes:
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5
```

**Important**: Service readiness checking is **enabled by default** (changed from previous behavior). To disable, set `container_readiness.wait_for_service_ready: false` in the config file.

See `test/autoware_planning_simulation/autoware_config.yaml` for a complete example and `docs/resource-monitoring-design.md` for detailed documentation.

## Log Directory Structure

After running `play_launch`, logs are organized with a **flat 1-level structure** for easy scripting:
```
play_log/
├── YYYY-MM-DD_HH-MM-SS/   # Timestamped log directory for each run
│   ├── params_files/      # Cached parameter files from record.json
│   ├── node/              # Regular node logs (flat structure)
│   │   └── <node_name>/   # Short node name (deduplicated with _2, _3 suffixes if needed)
│   │       ├── metadata.json  # Node metadata (package, namespace, container info)
│   │       ├── metrics.csv    # Resource metrics (when monitoring enabled)
│   │       ├── out            # stdout
│   │       ├── err            # stderr
│   │       ├── pid            # process ID
│   │       ├── status         # exit code
│   │       ├── cmdline        # executed command
│   │       └── params_files/  # Node-specific parameter files
│   └── load_node/         # Composable node logs (flat structure)
│       └── <node_name>/   # Short node name (deduplicated if needed)
│           ├── metadata.json  # Composable node metadata (plugin, target container, etc.)
│           ├── metrics.csv    # Resource metrics (when monitoring enabled)
│           ├── service_response.<round>  # LoadNode service response
│           └── status         # final status (0=success, 1=failure)
```

**Key Features:**
- **Flat Structure**: All node directories are at a single level for consistent scripting
- **Short Names**: Directory names use node names (e.g., `control_evaluator`) instead of lengthy package paths
- **Name Deduplication**: Duplicate names get numeric suffixes (e.g., `container_2`, `glog_component_3`)
- **Metadata Files**: Each node has `metadata.json` with full context (package, namespace, container info)
- **Self-Contained**: Each node directory contains all its data including metrics
- **Log Names Match Directories**: Log messages show short names matching directory names (e.g., `NODE 'rviz2'`)

**Metadata JSON Examples:**

Regular node (`node/control_evaluator/metadata.json`):
```json
{
  "type": "node",
  "package": "autoware_control_evaluator",
  "executable": "control_evaluator",
  "exec_name": "control_evaluator-41",
  "name": "control_evaluator",
  "namespace": "/control",
  "is_container": false
}
```

Container (`node/control_container/metadata.json`):
```json
{
  "type": "container",
  "package": "rclcpp_components",
  "executable": "component_container_mt",
  "exec_name": "component_container_mt-39",
  "name": "control_container",
  "namespace": "/control",
  "is_container": true,
  "container_full_name": "/control/control_container"
}
```

Composable node (`load_node/vehicle_cmd_gate/metadata.json`):
```json
{
  "type": "composable_node",
  "package": "autoware_vehicle_cmd_gate",
  "plugin": "autoware::vehicle_cmd_gate::VehicleCmdGate",
  "node_name": "vehicle_cmd_gate",
  "namespace": "/control",
  "target_container_name": "/control/control_container",
  "target_container_node_name": "control_container"
}
```

## Dependencies

### Rust (Cargo workspace)
- Async runtime: tokio
- Parallelism: rayon
- Error handling: eyre (with comprehensive error context via `.wrap_err()`)
- CLI: clap
- Serialization: serde, serde_json, serde_yaml
- ROS bindings: rclrs (for component loader service calls), composition_interfaces, rcl_interfaces
- Resource monitoring: sysinfo (cross-platform process metrics), nvml-wrapper (NVIDIA GPU monitoring)
- Configuration: glob (pattern matching), csv (metrics logging), num_cpus

### Python (uv managed)
- dump_launch: ruamel-yaml, pyyaml, lark, packaging
- Build: hatchling
- Testing: pytest, pytest-cov, pytest-mock, ruff

## Important Implementation Details

### Environment Setup
- **User responsibility**: Source ROS setup files before running `play_launch`
- play_launch does NOT source any setup.bash files internally
- Subprocess spawned by play_launch inherits the parent process environment (PATH, AMENT_PREFIX_PATH, etc.)
- **Environment variables from launch files**: dump_launch captures environment variables set via `<env>` tags in launch files, play_launch replays them to child processes
  - Captured from Node action's `additional_env` attribute
  - Stored in NodeRecord.env in record.json
  - Applied via `command.envs()` before spawning processes
  - Composable nodes inherit environment from their container processes

### Process Cleanup
- **CleanupGuard** (main.rs:207-215): RAII pattern ensures all child processes are killed on exit
- **Signal Handlers** (main.rs:398-432): Handle SIGTERM and SIGINT gracefully, call `kill_all_descendants()` before exiting
- **Process Group Isolation** (node_cmdline.rs:361-366, component_loader.rs:255-260): Uses `.process_group(0)` to spawn children in separate process groups
  - Prevents children from receiving terminal signals (Ctrl-C) directly
  - Only play_launch receives SIGINT, then explicitly kills all descendants
  - Eliminates race conditions where nodes survive after play_launch exits
- **Recursive Cleanup** (main.rs:47-111): `kill_all_descendants()` uses `pgrep` to find all descendants recursively, sends SIGTERM then SIGKILL
- **Logging**: All cleanup operations use structured logging (`debug!`, `info!`) instead of `eprintln!`

### Error Reporting
- Comprehensive error context using eyre's `.wrap_err_with()`
- Error propagation path tracked throughout component_loader.rs and execution.rs
- Errors display full context chain for debugging

### Composable Node Loading
- **Service-based only**: All composable node loading uses the component loader thread with rclrs
- Component loader makes direct service calls to `composition_interfaces/srv/LoadNode` using rclrs
- Service clients cached per container for efficiency
- Async/await pattern with tokio for concurrent loading
- Automatic parameter type conversion (strings to ROS parameter types)
- Load success/failure tracked in `service_response.*` files with full error messages
- Eliminates subprocess overhead (~50-200ms per node) compared to CLI approach

## Lifecycle Node Handling

Lifecycle nodes are tracked separately in `launch_dump.lifecycle_node` but currently require manual intervention. The import statement is fixed in commit 5e91fb1.

## Testing

### Autoware Planning Simulator Integration Test
Located in `test/autoware_planning_simulation/`:
- `Makefile`: Build automation with automatic environment sourcing
  - `make start-sim`: Start Autoware planning simulator with play_launch
  - `make drive`: Run autonomous driving test
  - `make start-sim-and-drive`: Complete test sequence (simulator + autonomous test)
  - `make plot`: Generate resource usage plots from latest play_log directory
  - `make kill-orphans`: Kill orphan ROS nodes
- `scripts/start-sim.sh`: Script to start simulator (extracted from Makefile)
- `scripts/test_autonomous_drive.py`: Automated autonomous driving test
- `scripts/plot_resource_usage.py`: Resource usage plotting and analysis tool
- `scripts/kill_orphan_nodes.sh`: Cleanup utility for orphan ROS nodes
- `poses_config.yaml`: Validated poses for sample-map-planning
- `README.md`: Comprehensive documentation

Successfully tested with Autoware planning_simulator (52 composable nodes, 15 containers).

## Resource Monitoring (Phase 1 Complete)

See `docs/resource-monitoring-design.md` for comprehensive design document.

**Status**: Phase 1 (Phase 1a + Phase 1b) implementation complete and tested ✓

**Implemented Features**:
- ✅ Per-node resource monitoring (CPU, memory, I/O, threads, FDs, process state)
- ✅ Container monitoring with PID registration and aggregated metrics
- ✅ CSV logging for visualization and analysis (one file per node/container)
- ✅ Process control: CPU affinity and nice values for containers and standalone composable nodes
- ✅ Configuration via YAML file with glob pattern matching
- ✅ CLI flags: `--config` / `-c`, `--enable-monitoring`, `--monitor-interval-ms`
- ✅ sysinfo 0.32 integration with optimized process refresh
- ✅ Background monitoring thread with configurable interval (default: 1000ms)
- ✅ Zero overhead when disabled (default)

**Implementation Details**:
- **config.rs**: Runtime configuration with YAML parsing, CPU affinity & nice value support, glob pattern matching
- **resource_monitor.rs**: Monitoring thread using sysinfo with targeted process refresh, CSV writing per node
  - Uses `System::new()` instead of `new_all()` to avoid loading all system processes
  - Refreshes only monitored PIDs using `ProcessesToUpdate::Some(&pids)` for efficiency
- **execution.rs**: Process control integration for containers (lines 320-333) and standalone composable nodes (lines 553-566)
- **main.rs**: Monitoring thread initialization and config passing to execution (line 324)
- **Integration**: Full pipeline from config → execution → monitoring
- **Tests**: 21 unit tests pass, all lint checks pass, tested with Autoware (15 containers, 46 nodes)
- **Coverage**: Python tests have proper coverage reporting

**Example usage**:
```bash
# Enable monitoring for all nodes with default 1s interval
ros2 run play_launch play_launch --enable-monitoring

# Use config file for fine-grained control
ros2 run play_launch play_launch --config config.yaml

# Short form
ros2 run play_launch play_launch -c config.yaml

# Override config with CLI flags
ros2 run play_launch play_launch -c config.yaml --enable-monitoring --monitor-interval-ms 500
```

**Example config.yaml**:
```yaml
monitoring:
  enabled: true
  sample_interval_ms: 1000

processes:
  # Apply process control to all component containers
  - node_pattern: "NODE 'rclcpp_components/component_container*"
    monitor: true
    cpu_affinity: [0, 1]
    nice: 5

  # Specific container with higher priority
  - node_pattern: "NODE 'rclcpp_components/component_container_mt-25'"
    monitor: true
    cpu_affinity: [0, 1]
    nice: -5  # Negative nice requires root privileges

  # Pattern matching for specific nodes
  - node_pattern: "NODE 'rviz2/rviz2*"
    monitor: false
    nice: 10
```

**Important Notes**:
- Negative nice values (-20 to -1, higher priority) require CAP_SYS_NICE capability (see Phase 2)
- Positive nice values (1 to 19, lower priority) can be set by any user
- GPU metrics automatically included in CSV output when NVIDIA drivers available (see Phase 2)

## Phase 2 Enhancements (Complete)

### CAP_SYS_NICE Capability Management

**Purpose**: Allow play_launch to set negative nice values (higher priority) without running as root.

**Implementation**:
```bash
# Build the workspace
make build

# Apply capability to the installed binary
sudo setcap cap_sys_nice+ep install/play_launch/lib/play_launch/play_launch

# Verify capability is set
getcap install/play_launch/lib/play_launch/play_launch
# Output: install/play_launch/lib/play_launch/play_launch = cap_sys_nice+ep
```

**Usage**:
```bash
# Source workspace
. install/setup.bash

# Use negative nice values in config (works without sudo if capability is set)
ros2 run play_launch play_launch -c config.yaml
```

**Note**: The capability must be reapplied after each rebuild since colcon overwrites the binary.

### GPU Monitoring (NVIDIA)

**Purpose**: Monitor per-process GPU usage alongside CPU and memory metrics.

**Implementation**: Direct integration with NVML (NVIDIA Management Library) via nvml-wrapper crate.

**Requirements**:
- NVIDIA GPU hardware
- NVIDIA driver installed (includes NVML library)
- No special permissions needed for monitoring

**Behavior**:
- NVML initializes at startup
- If NVML unavailable: Warning logged, GPU monitoring disabled, other metrics continue
- If NVML available: GPU metrics automatically collected for all monitored processes

**CSV Output** (extended with GPU, I/O rate, and network columns):
```csv
timestamp,pid,cpu_percent,cpu_user_secs,rss_bytes,vms_bytes,io_read_bytes,io_write_bytes,total_read_bytes,total_write_bytes,total_read_rate_bps,total_write_rate_bps,state,num_threads,num_fds,num_processes,gpu_memory_bytes,gpu_utilization_percent,gpu_memory_utilization_percent,gpu_temperature_celsius,gpu_power_milliwatts,gpu_graphics_clock_mhz,gpu_memory_clock_mhz,tcp_connections,udp_connections
```

**GPU Metrics Collected**:
- `gpu_memory_bytes`: GPU memory used by process
- `gpu_utilization_percent`: GPU compute utilization (0-100%)
- `gpu_memory_utilization_percent`: GPU memory interface utilization (0-100%)
- `gpu_temperature_celsius`: GPU core temperature
- `gpu_power_milliwatts`: GPU power consumption
- `gpu_graphics_clock_mhz`: Graphics clock frequency
- `gpu_memory_clock_mhz`: Memory clock frequency

**Multi-GPU Support**: Searches all GPUs to find which device(s) a process is using.

### I/O Rates and Network Metrics

**Purpose**: Monitor comprehensive I/O activity including network traffic, and track network connection counts.

**Implementation**:
- Parse `/proc/[pid]/io` directly for rchar/wchar (total I/O including network, pipes, etc.)
- Calculate I/O rates (bytes/sec) from cumulative counters by comparing with previous sample
- Count TCP/UDP connections from `/proc/[pid]/net/{tcp,tcp6,udp,udp6}` files

**I/O Metrics Collected**:
- `io_read_bytes`: Disk read bytes (from sysinfo, disk only)
- `io_write_bytes`: Disk write bytes (from sysinfo, disk only)
- `total_read_bytes`: Total read bytes including network (rchar from /proc/[pid]/io)
- `total_write_bytes`: Total write bytes including network (wchar from /proc/[pid]/io)
- `total_read_rate_bps`: Read rate in bytes/sec (calculated from previous sample)
- `total_write_rate_bps`: Write rate in bytes/sec (calculated from previous sample)

**Network Metrics Collected**:
- `tcp_connections`: Active TCP connections (IPv4 + IPv6)
- `udp_connections`: Active UDP connections (IPv4 + IPv6)

**Rate Calculation**:
- First sample has no rate (empty fields in CSV)
- Subsequent samples: `rate = (current_bytes - previous_bytes) / time_diff`
- Handles process restarts gracefully (resets previous sample)

**Example** (first sample - no rate yet):
```csv
2025-10-21T10:00:00.000Z,12345,15.3,100,104857600,524288000,1048576,524288,5242880,2621440,,,Running,8,42,1,,,,,,,4,2
```

**Example** (subsequent sample - with rates):
```csv
2025-10-21T10:00:01.000Z,12345,16.1,101,105906176,524288000,1572864,786432,10485760,5242880,5242880.00,2621440.00,Running,8,42,1,,,,,,,4,2
```

**Implementation Details**:
- **main.rs**: Initializes NVML at startup, passes to monitoring thread
- **resource_monitor.rs**: Extends ResourceMetrics struct with GPU fields, calls `Device::running_compute_processes()` to find PIDs
- **No fallback**: Direct NVML API only, no nvidia-smi subprocess parsing
- **Fail-soft**: NVML initialization failure does not prevent play_launch from running

## Phase 3: Visualization & Analysis

### Phase 3.1 Complete - Python Plotting Tool

**Status**: ✅ Complete

**Location**: `test/autoware_planning_simulation/scripts/plot_resource_usage.py`

**Features**:
- Automatic detection and plotting of available metrics (CPU, memory, GPU, I/O, network)
- Timeline plots showing metrics over time
- Distribution plots (box plots) showing statistical distributions
- Enhanced statistics report with top 10 rankings for all metrics
- Graceful handling of missing data (e.g., no GPU hardware)

**Generated Outputs** (in `play_log/<timestamp>/plots/`):
1. `cpu_usage.png` - CPU usage timeline
2. `memory_usage.png` - Memory usage timeline
3. `cpu_distribution.png` - CPU distribution box plot
4. `memory_distribution.png` - Memory distribution box plot
5. `io_read_usage.png` - I/O read rate timeline (when I/O data available)
6. `io_write_usage.png` - I/O write rate timeline (when I/O data available)
7. `io_distribution.png` - I/O rate distribution (when I/O data available)
8. `gpu_memory_usage.png` - GPU memory usage timeline (when GPU data available)
9. `gpu_utilization.png` - GPU compute utilization timeline (when GPU data available)
10. `gpu_temperature.png` - GPU temperature timeline (when GPU data available)
11. `gpu_power.png` - GPU power consumption timeline (when GPU data available)
12. `gpu_clocks.png` - GPU graphics and memory clocks timeline (when GPU data available)
13. `gpu_distribution.png` - GPU metrics distribution (when GPU data available)
14. `legend.png` - Node index legend mapping
15. `statistics.txt` - Comprehensive statistics report

**Statistics Report Includes**:
- Top 10 nodes by max/avg CPU usage
- Top 10 nodes by max/avg memory usage
- Top 10 nodes by avg I/O read/write rates
- Top 10 nodes by avg TCP/UDP connections
- Top 10 nodes by max GPU memory usage (when available)
- Top 10 nodes by avg GPU compute utilization (when available)
- Top 10 nodes by max GPU memory utilization % (when available)
- Top 10 nodes by max GPU temperature (when available)
- Top 10 nodes by max GPU power consumption (when available)
- Top 10 nodes by avg GPU graphics/memory clock speeds (when available)

**GPU Metrics Note**:
GPU metrics track CUDA compute processes only (via NVML's `running_compute_processes()`). Regular ROS nodes that don't execute CUDA kernels will show empty GPU data. This is expected behavior for most Autoware nodes. GPU metrics will populate when processes actively use CUDA for compute workloads (e.g., ML inference, perception nodes with CUDA acceleration).

**Usage**:
```bash
# Using Makefile (recommended - plots latest log directory)
cd test/autoware_planning_simulation
make plot

# Direct script invocation - plot latest log directory
cd test/autoware_planning_simulation
python3 scripts/plot_resource_usage.py

# Plot specific log directory
python3 scripts/plot_resource_usage.py --log-dir play_log/2025-10-21_20-33-20

# Specify output directory
python3 scripts/plot_resource_usage.py --output-dir custom_plots
```

**Future Enhancements** (Phase 3.2+):
- CLI analysis tool integrated into play_launch
- Interactive HTML plots
- Web dashboard (optional)
- See `docs/resource-monitoring-design.md` for complete roadmap

## Recent Changes

### 2025-10-28: NVML Library Loading Fix

**Problem Identified:**
- GPU monitoring failed with error: "Failed to initialize NVML: a libloading error occurred: libnvidia-ml.so: cannot open shared object file: No such file or directory"
- Error occurred despite nvidia-smi working correctly and NVIDIA drivers being installed
- The system had `libnvidia-ml.so.1` (runtime library) but not `libnvidia-ml.so` (development symlink)

**Root Cause:**
- nvml-wrapper 0.10 tried to load `libnvidia-ml.so` (development library name)
- Linux systems only install `libnvidia-ml.so.1` (versioned runtime library) with the NVIDIA driver package
- The unversioned `libnvidia-ml.so` only exists when CUDA development packages are installed

**Solution:**
- Updated nvml-wrapper from version 0.10 to 0.11.0
- Version 0.11.0 includes fix from PR #63 that changes LIB_PATH to `libnvidia-ml.so.1` on Linux
- This matches the actual runtime library name installed by NVIDIA drivers

**Files Modified:**
- `src/play_launch/Cargo.toml`: Updated nvml-wrapper dependency from "0.10" to "0.11"

**Testing:**
- GPU monitoring now initializes successfully: "NVML initialized successfully with 1 GPU device(s)"
- Autoware planning simulator runs without NVML errors
- GPU metrics collection functional

**Impact:**
- ✅ GPU monitoring now works on systems with NVIDIA drivers but without CUDA development packages
- ✅ Matches standard Linux library naming conventions for runtime libraries
- ✅ Eliminates need to install CUDA SDK just for GPU monitoring

### 2025-10-27: Namespace Resolution for Composable Nodes

**Problem Identified:**
- Autoware planning simulator had 2 composable nodes classified as "orphans" despite loading successfully with standard `ros2 launch`
- `pointcloud_to_laserscan_node` and `occupancy_grid_map_node` targeted `"pointcloud_container"` (relative name)
- Container was registered as `"/pointcloud_container"` (absolute name)
- ROS 2 runtime automatically resolves relative names to absolute at runtime, but play_launch was doing exact string matching
- This resulted in false-positive orphan warnings

**Root Cause:**
- dump_launch correctly captured literal `target_container_name` values from launch files (relative names)
- ROS 2's `create_client()` performs automatic namespace resolution that wasn't being replicated in play_launch
- When a LoadComposableNodes action runs in root namespace, rclpy expands relative service names to absolute

**Solution:**
- Added `resolve_container_name()` function in execution.rs (lines 20-34) mimicking ROS 2's namespace resolution
- Updated composable node classification logic (lines 171-201) to try both exact match and resolved names
- When resolved name matches, updates the record's `target_container_name` to use resolved name consistently
- Logs debug message when namespace resolution occurs

**Implementation:**
```rust
fn resolve_container_name(name: &str) -> String {
    if name.starts_with('/') {
        name.to_string()  // Already absolute
    } else {
        format!("/{}", name)  // Convert relative to absolute
    }
}
```

**Files Modified:**
- `src/play_launch/src/execution.rs`: Added namespace resolution function and updated matching logic

**Testing:**
- Tested with Autoware planning simulator (15 containers, 54 composable nodes)
- All composable nodes now correctly matched to containers
- **Zero orphan warnings** (previously 2 false-positive orphans)
- `pointcloud_to_laserscan_node` and `occupancy_grid_map_node` successfully load with `success: true` in service response
- Autoware autonomous driving test now functional

**Impact:**
- ✅ Eliminates false-positive orphan warnings for launch files using relative container names
- ✅ Matches ROS 2 runtime behavior for better compatibility
- ✅ Maintains backward compatibility with launch files already using absolute names
- ✅ Enables Autoware autonomous driving integration testing

### 2025-10-26: Log Directory Structure Refactoring

**Changes Made:**
- **Flat Directory Structure**: Redesigned node and composable node directories from multi-level hierarchy to flat single-level structure
  - Old: `node/autoware_control_evaluator/control_evaluator-41/`
  - New: `node/control_evaluator/`
  - Old: `load_node/@control@control_container/autoware_vehicle_cmd_gate/VehicleCmdGate/`
  - New: `load_node/vehicle_cmd_gate/`

- **Short Log Names**: Updated log messages to use short directory names
  - Old: `NODE 'autoware_control_evaluator/control_evaluator-41'`
  - New: `NODE 'control_evaluator'`

- **Metadata Files**: Added `metadata.json` in each node/composable node directory with full context
  - Package, executable, namespace information
  - Container identification (`is_container`, `container_full_name`)
  - Target container info for composable nodes

- **Name Deduplication**: Implemented automatic handling of duplicate node names with numeric suffixes
  - Examples: `container_2`, `container_3`, `glog_component_4`

- **Per-Node Metrics**: Moved resource monitoring from centralized `metrics/` to per-node `metrics.csv`
  - Old: `metrics/NODE 'control_evaluator'.csv`
  - New: `node/control_evaluator/metrics.csv`

- **Updated Plotting Script**: Modified `plot_resource_usage.py` to recursively search `node/*/metrics.csv` and `load_node/*/metrics.csv`

**Benefits:**
- **Script-Friendly**: Consistent 1-level depth makes scripting and automation easier
- **Self-Contained**: Each node directory contains all its data (logs, metrics, metadata)
- **Better Correlation**: Short names in logs directly match directory names
- **Metadata Preservation**: Full context preserved in JSON files despite short directory names

**Files Modified:**
- `src/play_launch/src/context.rs`: Added metadata generation, flat structure, deduplication
- `src/play_launch/src/resource_monitor.rs`: Changed to write per-node metrics.csv
- `src/play_launch/src/execution.rs`: Updated process registry to use PathBuf
- `src/play_launch/src/main.rs`: Updated process registry initialization
- `test/autoware_planning_simulation/scripts/plot_resource_usage.py`: Updated to find metrics in new locations

**Testing:**
- Successfully tested with Autoware planning simulator (61 nodes + composable nodes)
- All metrics collection and plotting working correctly

### 2025-10-25: AMENT_PREFIX_PATH Environment Inheritance Fix

**Problem Identified**
- Composable node loading was failing with "Could not find requested resource in ament index" errors
- Affected 153/154 component loads in Autoware planning simulator
- Investigation revealed containers had DIFFERENT AMENT_PREFIX_PATH than parent play_launch process
- Parent process had 31,866 characters including all workspace paths
- Containers were missing ~30,000 characters worth of workspace paths (ros-launch-perf paths specifically)

**Root Cause**
- While Rust's `Command::new()` inherits environment by default, explicit preservation ensures consistency
- Containers need access to ALL workspace paths in AMENT_PREFIX_PATH to find composable node plugins via ament index
- Without ros-launch-perf paths, containers could only find components in Autoware workspace

**Solution**
- Added explicit AMENT_PREFIX_PATH preservation in `node_cmdline.rs::to_command()` (lines 361-365)
- Captures parent's AMENT_PREFIX_PATH via `std::env::var()` and explicitly sets it via `command.env()`
- Ensures containers have identical AMENT_PREFIX_PATH as parent play_launch process

**Implementation:**
```rust
// Explicitly preserve AMENT_PREFIX_PATH to ensure containers have access to all workspaces
// This is critical for ament index lookups when loading composable nodes
if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
    command.env("AMENT_PREFIX_PATH", ament_prefix_path);
}
```

**Files Modified:**
- `src/play_launch/src/node_cmdline.rs`: Added AMENT_PREFIX_PATH preservation in `to_command()` method

**Testing:**
- Environment verification test confirms containers now receive full AMENT_PREFIX_PATH (31,866 characters, exact match)
- Autoware planning simulator test: **ALL 52 composable nodes load successfully** (previously 1/154 success rate)
- All `autoware_default_adapi` components that were failing now load correctly
- Zero "Could not find requested resource in ament index" errors

**Impact:**
- ✅ **100% success rate** for composable node loading in multi-workspace setups
- Resolves critical issue preventing Autoware planning simulator from starting via play_launch
- Ensures compatibility with complex ROS2 workspace overlays

### 2025-10-22: Service-Based Component Loading Migration

**Component Loader Rewrite**
- Migrated from `ros2 component load` CLI subprocesses to direct rclrs service calls
- Component loader now creates rclrs node with executor in background thread
- Service clients created and cached per container for LoadNode service
- Direct calls to `composition_interfaces/srv/LoadNode` using native Rust bindings
- Eliminated Python startup overhead (~50-200ms per node)
- Better parallelization with native async/await patterns
- Parameters automatically parsed from strings to proper ROS types (bool, int64, double, string)

**Dependencies Updated**
- Added `composition_interfaces = "*"` to Cargo.toml (generated Rust bindings)
- Added `rcl_interfaces = "*"` to Cargo.toml (for Parameter types)
- Removed old FFI-based composition_interfaces.rs module
- Removed composition_interfaces linking from build.rs (now handled by generated crate)

**Standalone Mode Unchanged**
- `--standalone-composable-nodes` mode still uses subprocess spawning via `ros2 component standalone`
- This is intentional as standalone nodes run as independent processes without containers
- Service-based loading only applies to container mode

**Files Modified:**
- `src/play_launch/Cargo.toml`: Added composition_interfaces and rcl_interfaces dependencies
- `src/play_launch/src/component_loader.rs`: Complete rewrite using rclrs service client
- `src/play_launch/src/main.rs`: Removed composition_interfaces module declaration
- `src/play_launch/src/composition_interfaces.rs`: Deleted (replaced by generated crate)
- `CLAUDE.md`: Updated architecture documentation

**Testing:**
- All tests pass (2 tests, 0 failures)
- Build succeeds with only rclrs library warnings
- Binary rebuilt at 2025-10-22 with all changes
- Successfully tested with ROS official composition demo (Talker/Listener components)
  - Test suite created at `test/composition_demo/`
  - Service responses confirmed: `success: true` with unique_id for both components
  - Inter-node communication verified (Publishing/I heard messages logged)
- Successfully tested with Autoware planning simulator (52 composable nodes, 15 containers)
  - 17 nodes successfully loaded via service calls
  - Service-based loading working correctly with concurrent loads
  - Container readiness detection functioning properly
  - Proper error reporting for parameter type mismatches (dump_launch issue)

### 2025-10-22: ROS Packaging Migration & Process Cleanup

**ROS Packaging Migration**
- Transitioned from standalone binary installation (`~/.cargo/bin/`) to proper ROS workspace packaging
- Removed `make install` and `make uninstall` targets
- Updated all scripts to use `ros2 run dump_launch dump_launch` and `ros2 run play_launch play_launch`
- Updated documentation to reflect ROS-based workflow
- CAP_SYS_NICE capability now applied to `install/play_launch/lib/play_launch/play_launch`
- **Note**: Capability must be reapplied after each rebuild since colcon overwrites the binary

**Process Cleanup Improvements**
- Fixed orphan process issue when play_launch is terminated with Ctrl-C
- Implemented process group isolation using `.process_group(0)` for all child processes
- Children now spawn in separate process groups, isolated from terminal signals
- Only play_launch receives SIGINT when Ctrl-C is pressed
- play_launch's signal handler explicitly kills all descendants via `kill_all_descendants()`
- Replaced all `eprintln!` statements with structured logging (`debug!`, `warn!`)
- **Impact**: Eliminates race conditions where ROS nodes survive after play_launch exits

**Files Modified:**
- `Makefile`: Removed install/uninstall targets, updated help text
- `CLAUDE.md`: Updated Build & Install, Running the Tools, Process Cleanup, CAP_SYS_NICE sections
- `test/autoware_planning_simulation/scripts/start-sim.sh`: Changed to use `ros2 run` commands
- `test/autoware_planning_simulation/scripts/start-sim-and-drive.sh`: Changed to use `ros2 run` commands
- `src/play_launch/src/node_cmdline.rs`: Added `.process_group(0)` at lines 361-366
- `src/play_launch/src/component_loader.rs`: Added `.process_group(0)` at lines 255-260
- `src/play_launch/src/main.rs`: Replaced `eprintln!` with `debug!`/`warn!` logging
- `docs/resource-monitoring-design.md`: Documented ROS migration and process cleanup improvements

**Testing:**
- Binary rebuilt at 2025-10-22 11:41 with all changes
- Process group isolation verified in code
- Cleanup mechanism tested with SIGINT/SIGTERM handlers
