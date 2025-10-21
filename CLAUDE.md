# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Launch Inspection Tool - A dual-component system for recording and replaying ROS 2 launch executions. The project consists of:

1. **dump_launch** (Python): Records ROS 2 launch file execution and generates `record.json`
2. **play_launch** (Rust): Replays the recorded launch execution from `record.json`

## Build & Install

### Full Installation
```sh
make install
```
This runs `uv sync && uv build` for Python and `cargo build --release --all-targets` for Rust, then installs both components.

### Build Only
```sh
make build
```

### Uninstall
```sh
make uninstall
```

### Debian Package
```sh
make debian  # or: cargo deb
```

### Clean
```sh
make clean
```

## Development Commands

### Running the Tools

Record a launch execution (replace `ros2 launch` with `dump_launch`):
```sh
dump_launch <package> <launch_file> [args...]
```

Replay the recorded launch:
```sh
play_launch
```

Generate shell script from record:
```sh
play_launch --print-shell > launch.sh
```

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
   - Background thread started with ROS context for service-based node loading
   - Uses `ros2 component load` CLI via subprocess (avoiding shell escaping issues)
   - Future optimization: direct RCL service calls to composition_interfaces/srv/LoadNode

4. **Execution** (execution.rs):
   - **Regular nodes**: Spawned directly via `spawn_nodes()`
   - **Composable nodes**: Two execution modes:
     - **Standalone mode** (`--standalone-composable-nodes`): Each composable node runs in its own process via `ros2 component standalone`
     - **Container mode** (default): Containers spawn first, then composable nodes load via service calls

5. **Composable Node Loading Strategy** (service-based only):
   - Nodes classified as "nice" (have matching container) or "orphan" (no matching container)
   - Loading is concurrent with configurable `max_concurrent_load_node_spawn` (default: 10)
   - Component loader thread handles service calls to `composition_interfaces/srv/LoadNode`
   - Retry logic: up to `load_node_attempts` (default: 3) with `load_node_timeout_millis` (default: 30s)
   - Orphans only loaded if `--load-orphan-composable-nodes` is set

6. **Container Readiness Checking** (optional, container_readiness.rs):
   - Enable with `--wait-for-service-ready` flag
   - ROS service discovery thread monitors container services
   - Waits for `list_nodes` and `load_node` services to be available
   - Configurable timeout via `--service-ready-timeout-secs` (default: 120s)

7. **Logging**: All node stdout/stderr, PIDs, status codes saved to `play_log/node/` and `play_log/load_node/`

### Key Rust Modules

- **main.rs**: Entry point, orchestrates the async runtime and execution flow, includes CleanupGuard for subprocess cleanup
- **launch_dump.rs**: Deserialization and data transformation
- **context.rs**: Execution context preparation for nodes and composable nodes
- **execution.rs**: Process spawning, container management, composable node loading via service calls
- **component_loader.rs**: Background thread for service-based composable node loading
- **composition_interfaces.rs**: FFI bindings for composition_interfaces/srv/LoadNode (prepared for future direct RCL calls)
- **container_readiness.rs**: Optional service discovery and readiness checking for containers
- **node_cmdline.rs**: Command-line parsing and generation for ROS nodes
- **options.rs**: CLI argument parsing

### Python dump_launch

- **inspector.py**: Core inspection logic for ROS launch files
- **event_handlers.py**: Launch event handling
- **ros_cmdline/**: ROS command-line utilities

## Important Configuration Options

play_launch accepts these options (play_launch/src/options.rs):

### Basic Options
- `--log-dir <PATH>`: Log directory (default: `play_log`)
- `--input-file <PATH>`: Input record file (default: `record.json`)
- `--print-shell`: Generate shell script instead of executing

### Runtime Configuration
- `--config <PATH>` (short: `-c`): Runtime configuration file (YAML)
  - Resource monitoring settings
  - Per-process CPU affinity and nice values
  - See `docs/resource-monitoring-design.md` for details

### Resource Monitoring
- `--enable-monitoring`: Enable resource monitoring for all nodes (overrides config file)
- `--monitor-interval-ms <MS>`: Sampling interval in milliseconds (overrides config file)

### Composable Node Loading
- `--delay-load-node-millis <MS>`: Delay before loading composable nodes (default: 2000ms)
- `--load-node-timeout-millis <MS>`: Timeout for composable node loading (default: 30000ms)
- `--load-node-attempts <N>`: Max retry attempts (default: 3)
- `--max-concurrent-load-node-spawn <N>`: Concurrent loading limit (default: 10)
- `--standalone-composable-nodes`: Run composable nodes standalone instead of loading into containers
- `--load-orphan-composable-nodes`: Load composable nodes that have no matching container

### Container Readiness
- `--wait-for-service-ready`: Enable container service readiness checking via ROS service discovery
- `--service-ready-timeout-secs <N>`: Max wait time for container services (default: 120s, 0=unlimited)
- `--service-poll-interval-ms <MS>`: Polling interval for service discovery (default: 500ms)

## Log Directory Structure

After running `play_launch`, logs are organized as:
```
play_log/
├── YYYY-MM-DD_HH-MM-SS/   # Timestamped log directory for each run
│   ├── params_files/      # Cached parameter files from record.json
│   ├── node/              # Regular node logs
│   │   └── <package>/<exec_name>/
│   │       ├── out        # stdout
│   │       ├── err        # stderr
│   │       ├── pid        # process ID
│   │       ├── status     # exit code
│   │       └── cmdline    # executed command
│   ├── load_node/         # Composable node logs
│   │   └── <container>/<package>/<plugin>/
│   │       ├── service_response.<round>  # LoadNode service response
│   │       └── status                    # final status (0=success, 1=failure)
│   └── metrics/           # Resource monitoring (when enabled)
│       └── <node_name>.csv  # Per-node/container resource metrics
│           # Columns: timestamp, pid, cpu_percent, cpu_user_secs, rss_bytes, vms_bytes,
│           #          io_read_bytes, io_write_bytes, state, num_threads, num_fds, num_processes
```

## Dependencies

### Rust (Cargo workspace)
- Async runtime: tokio
- Parallelism: rayon
- Error handling: eyre (with comprehensive error context via `.wrap_err()`)
- CLI: clap
- Serialization: serde, serde_json, serde_yaml
- ROS bindings: rclrs (for component loader ROS context)
- Resource monitoring: sysinfo (cross-platform process metrics)
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

### Process Cleanup
- **CleanupGuard** (main.rs): RAII pattern ensures all child processes are killed on exit
- Handles SIGTERM and SIGINT gracefully
- Recursively kills all descendant processes using `pgrep` and `kill`

### Error Reporting
- Comprehensive error context using eyre's `.wrap_err_with()`
- Error propagation path tracked throughout component_loader.rs and execution.rs
- Errors display full context chain for debugging

### Composable Node Loading
- **Service-based only**: All composable node loading uses the component loader thread
- Component loader calls `ros2 component load` CLI (not bash wrapper to avoid escaping issues)
- Future optimization: Direct RCL service calls (FFI bindings prepared in composition_interfaces.rs)
- Load success/failure tracked in `service_response.*` files

## Lifecycle Node Handling

Lifecycle nodes are tracked separately in `launch_dump.lifecycle_node` but currently require manual intervention. The import statement is fixed in commit 5e91fb1.

## Testing

### Autoware Integration Test
Located in `scripts/autoware_test/`:
- `Makefile`: Build automation with automatic environment sourcing
  - `make run`: Start Autoware with play_launch
  - `make test`: Run autonomous driving test
  - `make full-test`: Complete test sequence
  - `make clean`: Kill orphan ROS nodes
- `test_autonomous_drive.py`: Automated autonomous driving test
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
play_launch --enable-monitoring

# Use config file for fine-grained control
play_launch --config config.yaml

# Short form
play_launch -c config.yaml

# Override config with CLI flags
play_launch -c config.yaml --enable-monitoring --monitor-interval-ms 500
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
- Negative nice values (-20 to -1, higher priority) require root privileges
- Positive nice values (1 to 19, lower priority) can be set by any user
- CPU affinity is an array of core IDs (e.g., `[0, 1]` pins to cores 0 and 1)
- Pattern matching uses glob syntax (`*` for wildcards)
- Node names follow the format: `NODE '<package>/<executable>-<id>'`
- First matching pattern is applied (order matters)

**Next Steps (Phase 2)**:
- GPU metrics (NVIDIA/AMD)
- Network I/O statistics
- Aggregated container-level metrics
- See design doc for complete Phase 2 & 3 roadmap
