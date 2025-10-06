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

1. **Load & Transform** (launch_dump.rs): Deserialize `record.json` and copy cached parameter files to `log/params_files/`

2. **Context Preparation** (context.rs):
   - `prepare_node_contexts()`: Classify nodes into containers vs regular nodes
   - `prepare_composable_node_contexts()`: Prepare composable node loading contexts

3. **Execution** (execution.rs):
   - **Regular nodes**: Spawned directly via `spawn_nodes()`
   - **Composable nodes**: Two execution modes:
     - **Standalone mode** (`--standalone-composable-nodes`): Each composable node runs in its own process via `ros2 component standalone`
     - **Container mode** (default): Containers spawn first, then composable nodes load into them via `ros2 component load`

4. **Composable Node Loading Strategy**:
   - Nodes classified as "nice" (have matching container) or "orphan" (no matching container)
   - Loading is concurrent with configurable `max_concurrent_load_node_spawn` (default: 10)
   - Retry logic: up to `load_node_attempts` (default: 3) with `load_node_timeout_millis` (default: 30s)
   - Orphans only loaded if `--load-orphan-composable-nodes` is set

5. **Logging**: All node stdout/stderr, PIDs, status codes saved to `log/node/` and `log/load_node/`

### Key Rust Modules

- **main.rs**: Entry point, orchestrates the async runtime and execution flow
- **launch_dump.rs**: Deserialization and data transformation
- **context.rs**: Execution context preparation for nodes and composable nodes
- **execution.rs**: Process spawning, container management, composable node loading
- **node_cmdline.rs**: Command-line parsing and generation for ROS nodes
- **options.rs**: CLI argument parsing

### Python dump_launch

- **inspector.py**: Core inspection logic for ROS launch files
- **event_handlers.py**: Launch event handling
- **ros_cmdline/**: ROS command-line utilities

## Important Configuration Options

play_launch accepts these options (play_launch/src/options.rs):
- `--log-dir <PATH>`: Log directory (default: `log`)
- `--input-file <PATH>`: Input record file (default: `record.json`)
- `--delay-load-node-millis <MS>`: Delay before loading composable nodes (default: 100ms)
- `--load-node-timeout-millis <MS>`: Timeout for composable node loading (default: 30s)
- `--load-node-attempts <N>`: Max retry attempts (default: 3)
- `--max-concurrent-load-node-spawn <N>`: Concurrent loading limit (default: 10)
- `--standalone-composable-nodes`: Run composable nodes standalone instead of loading into containers
- `--load-orphan-composable-nodes`: Load composable nodes that have no matching container
- `--print-shell`: Generate shell script instead of executing

## Log Directory Structure

After running `play_launch`, logs are organized as:
```
log/
├── params_files/          # Cached parameter files from record.json
├── node/                  # Regular node logs
│   └── <package>/<exec_name>/
│       ├── out            # stdout
│       ├── err            # stderr
│       ├── pid            # process ID
│       ├── status         # exit code
│       └── cmdline        # executed command
└── load_node/             # Composable node logs
    └── <container>/<package>/<plugin>/
        ├── out.<round>    # stdout (per attempt)
        ├── err.<round>    # stderr (per attempt)
        ├── pid.<round>    # PID (per attempt)
        └── status         # final exit code
```

## Dependencies

### Rust (Cargo workspace)
- Async runtime: tokio
- Parallelism: rayon
- Error handling: eyre
- CLI: clap
- Serialization: serde, serde_json

### Python (uv managed)
- dump_launch: ruamel-yaml, pyyaml, lark, packaging
- Build: hatchling

## Lifecycle Node Handling

Lifecycle nodes are tracked separately in `launch_dump.lifecycle_node` but currently require manual intervention. The import statement is fixed in commit 5e91fb1.
