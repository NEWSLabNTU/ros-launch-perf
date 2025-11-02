# play_launch CLI Roadmap

## Overview

This document outlines the implementation roadmap for the unified `play_launch` CLI interface that provides a seamless user experience similar to `ros2 launch` and `ros2 run` commands. The CLI uses a subcommand-based architecture for clear, intuitive operation.

## Goals

1. **Familiar UX**: ROS 2 users can use `play_launch` like native ros2 commands
2. **Transparent workflow**: Automatic dump-and-replay in a single command
3. **Flexible**: Support dump-only and replay-only modes for advanced users
4. **Production ready**: Robust error handling and comprehensive testing

---

## Current Status

### ✅ Phase 1: Core Subcommand Structure (COMPLETE)

The subcommand-based CLI interface has been fully implemented with the following structure:

```bash
play_launch <COMMAND>

Commands:
  launch  Launch a ROS 2 launch file (dump + replay)
  run     Run a single ROS 2 node (dump + replay)
  dump    Dump launch execution without replaying
  replay  Replay from existing record.json
  help    Print this message or the help of the given subcommand(s)
```

**Implemented Changes**:

1. **options.rs**:
   - Command enum with Launch, Run, Dump, Replay variants
   - ReplayArgs includes CommonOptions (flattened)
   - LaunchArgs, RunArgs, DumpArgs structures defined
   - `#[command(arg_required_else_help = true)]` enforces subcommand requirement

2. **main.rs**:
   - Subcommand routing implemented
   - Replay subcommand fully functional
   - Launch/Run/Dump stubs return "not yet implemented" errors with migration instructions

3. **CLI Behavior**:
   - Running `play_launch` without arguments shows help message
   - `play_launch replay` fully functional with all existing features
   - All replay options available: monitoring, service readiness, container loading, etc.

**Test Status**:
- ✅ CLI parses all subcommands correctly
- ✅ Help messages are comprehensive and clear
- ✅ Replay subcommand works with all options
- ✅ Autoware test scripts updated to use new CLI

---

## ✅ Phase 2: dump_launch Integration (COMPLETE)

**Goal**: Implement automatic invocation of `dump_launch` for `launch` and `run` subcommands to provide seamless one-command workflow.

**Status**: ✅ Complete

**Completed**: 2025-10-29

### Work Items

#### 2.1 Create dump_launch Invocation Module

**File**: `src/play_launch/src/dump_launcher.rs` (NEW)

**Purpose**: Wrapper for invoking `dump_launch` as a subprocess and managing temporary record files.

**Key Functions**:
```rust
pub struct DumpLauncher {
    dump_launch_path: PathBuf,
}

impl DumpLauncher {
    /// Find dump_launch via 'which' command (PATH + ROS environment)
    pub fn new() -> eyre::Result<Self>;

    /// Execute dump_launch for a launch file
    pub async fn dump_launch(
        &self,
        package_or_path: &str,
        launch_file: Option<&str>,
        args: &[String],
        output: &Path,
    ) -> eyre::Result<()>;

    /// Execute dump_launch for a single node
    pub async fn dump_run(
        &self,
        package: &str,
        executable: &str,
        args: &[String],
        output: &Path,
    ) -> eyre::Result<()>;
}
```

**Implementation Details**:
- Use `which` crate to find `dump_launch` binary in PATH
- Build command: `ros2 run dump_launch dump_launch <args>`
- Execute via tokio `Command` for async subprocess management
- Stream stdout/stderr to user in real-time
- Handle errors from dump_launch subprocess
- Validate that output file was created successfully

**Dependencies**:
```toml
[dependencies]
which = "6.0"  # For finding binaries in PATH
```

**Error Handling**:
- Clear error if dump_launch not found: "dump_launch not found in PATH. Ensure ROS workspace is sourced."
- Pass through dump_launch errors with context
- Validate record.json after dump completes

**Testing**:
- [ ] Finds dump_launch when ROS workspace is sourced
- [ ] Returns clear error when workspace not sourced
- [ ] Correctly builds command for package-based launch
- [ ] Correctly builds command for path-based launch
- [ ] Handles launch arguments with special characters
- [ ] Streams output to user in real-time
- [ ] Errors from dump_launch are properly reported
- [ ] Generated record.json is valid

#### 2.2 Implement Launch Subcommand

**File**: `src/play_launch/src/main.rs` - `handle_launch()`

**Workflow**:
1. Invoke dump_launch to generate record.json
2. Wait for dump completion
3. Validate record.json exists and is valid
4. Call handle_replay() with generated record

**Implementation**:
```rust
fn handle_launch(args: &LaunchArgs) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()
        .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

    // Use default record.json or temporary file
    let record_path = PathBuf::from("record.json");

    info!("Step 1/2: Recording launch execution...");
    launcher.dump_launch(
        &args.package_or_path,
        args.launch_file.as_deref(),
        &args.launch_arguments,
        &record_path,
    )?;

    info!("Step 2/2: Replaying launch execution...");
    let replay_args = ReplayArgs {
        input_file: record_path,
        common: CommonOptions::default(), // or merge from args
    };
    handle_replay(&replay_args)?;

    Ok(())
}
```

**Future Enhancement**: Add `--keep-record` flag to preserve record.json after replay

**Testing**:
- [ ] `play_launch launch demo_nodes_cpp talker_listener.launch.py` works end-to-end
- [ ] Launch arguments passed correctly (e.g., `use_sim_time:=true`)
- [ ] Errors during dump prevent replay from starting
- [ ] Errors during replay are reported clearly
- [ ] Works with package names: `play_launch launch demo_nodes_cpp file.py`
- [ ] Works with file paths: `play_launch launch /path/to/file.launch.py`
- [ ] Works with Autoware: `play_launch launch autoware_launch planning_simulator.launch.xml map_path:=...`

#### 2.3 Implement Run Subcommand

**File**: `src/play_launch/src/main.rs` - `handle_run()`

**Workflow**:
1. Invoke dump_launch in "run" mode (single node)
2. Replay the single node

**Implementation**:
```rust
fn handle_run(args: &RunArgs) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()?;
    let record_path = PathBuf::from("record.json");

    info!("Step 1/2: Recording node execution...");
    launcher.dump_run(
        &args.package,
        &args.executable,
        &args.args,
        &record_path,
    )?;

    info!("Step 2/2: Replaying node execution...");
    let replay_args = ReplayArgs {
        input_file: record_path,
        common: CommonOptions::default(),
    };
    handle_replay(&replay_args)?;

    Ok(())
}
```

**Testing**:
- [ ] `play_launch run demo_nodes_cpp talker` works
- [ ] Node arguments passed correctly: `play_launch run demo_nodes_cpp talker --ros-args -p topic:=chatter`
- [ ] Single node execution verified

#### 2.4 Implement Dump Subcommand

**File**: `src/play_launch/src/main.rs` - `handle_dump()`

**Purpose**: Dump-only mode (no replay) for advanced users who want to inspect or modify record.json

**Implementation**:
```rust
fn handle_dump(args: &DumpArgs) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()?;

    match &args.subcommand {
        DumpSubcommand::Launch(launch_args) => {
            launcher.dump_launch(
                &launch_args.package_or_path,
                launch_args.launch_file.as_deref(),
                &launch_args.launch_arguments,
                &args.output,
            )?;
        }
        DumpSubcommand::Run(run_args) => {
            launcher.dump_run(
                &run_args.package,
                &run_args.executable,
                &run_args.args,
                &args.output,
            )?;
        }
    }

    info!("Dump completed: {}", args.output.display());
    info!("To replay: play_launch replay --input-file {}", args.output.display());
    Ok(())
}
```

**Testing**:
- [ ] `play_launch dump launch demo_nodes_cpp file.py --output test.json` creates test.json
- [ ] `play_launch dump run demo_nodes_cpp talker --output talker.json` creates talker.json
- [ ] No replay is executed
- [ ] Custom output paths work correctly
- [ ] Debug flag (`--debug`) passes through to dump_launch

---

## ✅ Phase 2.5: CLI Simplification (COMPLETE)

**Goal**: Simplify the CLI interface by moving fine-grained tuning parameters to configuration file, making service readiness the default, and removing the print-shell feature.

**Status**: ✅ Complete

**Completed**: 2025-10-29

### Implemented Changes

#### Configuration Structure (config.rs)
- ✅ Added `composable_node_loading` section with 4 parameters:
  - `delay_load_node_millis` (default: 2000ms)
  - `load_node_timeout_millis` (default: 30000ms)
  - `load_node_attempts` (default: 3)
  - `max_concurrent_load_node_spawn` (default: 10)

- ✅ Added `container_readiness` section with 3 parameters:
  - `wait_for_service_ready` (default: **true** - changed from false!)
  - `service_ready_timeout_secs` (default: 120s)
  - `service_poll_interval_ms` (default: 500ms)

#### CLI Options (options.rs)
- ✅ Removed 8 CLI flags (moved to config file):
  - `--print-shell` (removed entirely, including supporting functions)
  - `--wait-for-service-ready`
  - `--service-ready-timeout-secs`
  - `--service-poll-interval-ms`
  - `--delay-load-node-millis`
  - `--load-node-timeout-millis`
  - `--load-node-attempts`
  - `--max-concurrent-load-node-spawn`

- ✅ Kept 6 essential CLI flags:
  - `--log-dir` (essential output control)
  - `--config / -c` (primary interface for fine-grained control)
  - `--enable-monitoring` (feature toggle)
  - `--monitor-interval-ms` (useful override)
  - `--standalone-composable-nodes` (behavioral toggle)
  - `--load-orphan-composable-nodes` (behavioral toggle)

#### Code Updates
- ✅ Removed `generate_shell()` function from main.rs
- ✅ Removed `load_and_transform_node_records()` from launch_dump.rs
- ✅ Removed `copy_cached_data()` from launch_dump.rs
- ✅ Removed `NodeCommandLine::from_cmdline()` method
- ✅ Removed 8 tests that tested removed functionality
- ✅ Updated `handle_replay()` to use config values
- ✅ Updated `play()` to use config for composable node loading

#### Example Configuration
- ✅ Created `test/autoware_planning_simulation/autoware_config.yaml` with:
  - Autoware-optimized timeouts (60s for node loading, 300s for service ready)
  - Comprehensive comments explaining each setting
  - Process-specific examples (commented out)

#### Test Scripts
- ✅ Updated `start-sim.sh` and `start-sim-and-drive.sh` to use simplified CLI:
  ```bash
  play_launch launch autoware_launch planning_simulator.launch.xml \
      map_path:="$MAP_PATH" \
      --enable-monitoring \
      --monitor-interval-ms 1000 \
      --config autoware_config.yaml
  ```

#### Documentation
- ✅ Updated CLAUDE.md with:
  - Removed print-shell documentation
  - New "CLI Flags (Simplified)" section
  - New "Configuration File (config.yaml)" section with examples
  - Updated container readiness docs (now default: enabled)

### Impact
**Before**: 14+ CLI flags made the interface overwhelming
**After**: 6 essential CLI flags + config file for advanced users

**Key Change**: Service readiness checking is now **enabled by default**, improving reliability out-of-the-box while still allowing users to disable it via config file if needed.

---

## Phase 3: Documentation & Polish

**Goal**: Update all documentation and ensure production-ready user experience.

**Status**: 🔄 Partially Complete

**Estimated Effort**: 1 day

### Work Items

#### 3.1 Update CLAUDE.md

**File**: `CLAUDE.md`

**Status**: ✅ Partially Updated (need to update after Phase 2 completion)

**Changes Needed**:
- [x] Document new subcommand structure
- [x] Update "Running the Tools" examples
- [ ] Add examples for all four subcommands
- [ ] Update Architecture section with dump_launcher module
- [ ] Document the automatic dump-and-replay workflow

#### 3.2 Update Test Scripts

**Status**: ✅ Complete

**Updated Files**:
- [x] `test/autoware_planning_simulation/scripts/start-sim.sh`
- [x] `test/autoware_planning_simulation/scripts/start-sim-and-drive.sh`

**Changes**:
- Updated to use `play_launch replay` subcommand
- Added TODO comments for future migration to `play_launch launch` (Phase 2)

#### 3.3 Update CLI Help Text

**File**: `src/play_launch/src/options.rs`

**Status**: ✅ Complete

**Implemented**:
- [x] Comprehensive help text for main command
- [x] Subcommand descriptions
- [x] Example commands in `after_help`
- [x] Help text for all options

**Verify**:
- [x] `play_launch --help` shows main help
- [x] `play_launch replay --help` shows all replay options
- [x] Examples are accurate and helpful

#### 3.4 Create CLI Interface Documentation

**File**: `docs/cli-interface.md`

**Status**: ⏳ TODO (create after Phase 2 completion)

**Content**:
- Command reference for all subcommands
- Option descriptions
- Usage examples
- Common workflows
- Troubleshooting guide

---

## Phase 4: Testing & Validation

**Goal**: Comprehensive testing across multiple scenarios.

**Status**: 🔄 Partially Complete

**Estimated Effort**: 2-3 days

### Current Test Coverage

#### ✅ Replay Subcommand Tests
- [x] Parses all options correctly
- [x] Default input file (record.json) works
- [x] Custom input file (`--input-file`) works
- [x] Monitoring options work (`--enable-monitoring`, `--monitor-interval-ms`)
- [x] Service readiness works (`--wait-for-service-ready`)
- [x] Container loading options work
- [x] Tested with Autoware (15 containers, 54 composable nodes)
- [x] Tested with demo_nodes_cpp

#### ⏳ Integration Tests (Pending Phase 2)

**Test Scenarios**:
- [ ] `play_launch launch demo_nodes_cpp talker_listener.launch.py`
  - Nodes start correctly
  - Communication verified
  - Logs saved correctly
- [ ] `play_launch run demo_nodes_cpp talker`
  - Single node execution
  - Node outputs verified
- [ ] `play_launch dump launch demo_nodes_cpp talker_listener.launch.py --output test.json`
  - test.json created
  - JSON structure valid
  - No replay execution
- [ ] `play_launch launch autoware_launch planning_simulator.launch.xml map_path:=...`
  - All containers start
  - All composable nodes load
  - Autonomous driving test passes

#### Error Handling Tests

**Critical Scenarios**:
- [ ] dump_launch not in PATH
  - Clear error: "dump_launch not found in PATH. Ensure ROS workspace is sourced."
- [ ] Invalid package name
  - Error from dump_launch displayed
- [ ] Invalid launch file
  - Error from dump_launch displayed
- [ ] Malformed record.json
  - Parsing error with clear message
- [ ] Ctrl-C during dump
  - Cleanup happens correctly
  - Partial record.json not used
- [ ] Ctrl-C during replay
  - All processes killed
  - CleanupGuard works correctly

#### Edge Cases
- [ ] Launch file paths with spaces
- [ ] Very long argument lists
- [ ] Special characters in arguments (`:=`, quotes, etc.)
- [ ] Relative vs absolute paths
- [ ] Symlinked launch files

---

## Phase 5: Optional Enhancements

**Goal**: Quality-of-life improvements.

**Status**: ⏳ Future Work

**Estimated Effort**: 1-2 days

### Work Items

#### 5.1 Environment Variables

**Implementation**:
```bash
export PLAY_LAUNCH_RECORD_FILE="custom.json"
export PLAY_LAUNCH_LOG_DIR="custom_logs"
export PLAY_LAUNCH_CONFIG="custom_config.yaml"
```

**Behavior**: CLI arguments override environment variables

#### 5.2 Shell Completion

**Files**:
- `completions/play_launch.bash`
- `completions/play_launch.zsh`

**Implementation**: Use clap's `generate` feature

#### 5.3 Progress Indicators

**Dependencies**: `indicatif` crate

**Features**:
- Show progress during dump phase
- Show progress during composable node loading
- Spinner for long operations

#### 5.4 Colored Output

**Dependencies**: `colored` crate

**Features**:
- Color-coded messages (info=blue, warn=yellow, error=red)
- Auto-disable in non-TTY environments

#### 5.5 Keep Record Option

**Flag**: `--keep-record` for launch/run subcommands

**Behavior**: Don't delete record.json after successful replay

---

## Success Criteria

### Phase 1 (Complete) ✅
- [x] Subcommand structure implemented
- [x] CLI requires subcommand
- [x] Help messages comprehensive
- [x] Replay subcommand fully functional
- [x] Test scripts updated

### Phase 2 (In Progress) 🔄
- [ ] dump_launch integration working
- [ ] launch subcommand functional
- [ ] run subcommand functional
- [ ] dump subcommand functional
- [ ] End-to-end tests pass

### Phase 3 (Partial) ⏳
- [x] CLI help text complete
- [ ] CLAUDE.md updated (after Phase 2)
- [ ] CLI interface documentation created

### Phase 4 (Pending) ⏳
- [ ] Integration tests pass
- [ ] Error handling robust
- [ ] Edge cases handled
- [ ] Autoware test passes with new CLI

---

## Timeline Estimate

| Phase | Days | Status | Dependencies |
|-------|------|--------|--------------|
| Phase 1 | 2-3 | ✅ Complete | None |
| Phase 2 | 2-3 | ⏳ TODO | Phase 1 |
| Phase 3 | 1 | 🔄 Partial | Phase 2 |
| Phase 4 | 2-3 | ⏳ TODO | Phases 2-3 |
| **Total (MVP)** | **7-10 days** | **~30% Complete** | |
| Phase 5 (optional) | 1-2 | ⏳ Future | MVP |

**Current Progress**: Phase 1 complete (~30% of MVP)

---

## Dependencies

### External
- `dump_launch` package (already present)
- ROS 2 workspace must be sourced for dump_launch to be in PATH

### Rust Crates
- ✅ `clap` (already present) - CLI parsing
- ✅ `tokio` (already present) - Async runtime
- ✅ `eyre` (already present) - Error handling
- ⏳ `which` (to be added) - Finding binaries in PATH

---

## Migration Notes

### For Existing Scripts

**Before** (current two-step workflow):
```bash
ros2 run dump_launch dump_launch autoware_launch planning_simulator.launch.xml map_path:=...
play_launch replay --enable-monitoring --wait-for-service-ready
```

**After Phase 2** (one-step workflow):
```bash
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=... \
    --enable-monitoring \
    --wait-for-service-ready
```

### Backward Compatibility

- The two-step workflow will continue to work
- `play_launch replay` is fully functional
- Existing test scripts have been updated but old syntax still works

---

## Phase 6: I/O Helper Integration

**Goal**: Replace direct `/proc/[pid]/io` reads with privileged helper daemon to enable I/O monitoring for processes with capabilities set (containers, privileged nodes).

**Status**: ⏳ Planned

**Estimated Effort**: 2-3 days

### Background

Current I/O monitoring reads `/proc/[pid]/io` directly, which fails with EPERM (Permission Denied) when monitoring processes that have file capabilities set. This affects ROS 2 containers and nodes with elevated privileges, as the kernel clears the "dumpable" flag for security.

**Solution**: A separate helper daemon (`play_launch_io_helper`) with `CAP_SYS_PTRACE` capability can read these files. Communication uses anonymous pipes for security and automatic cleanup.

**Infrastructure**: ✅ Complete (Phase 5.5)
- Helper daemon binary built and installed
- IPC protocol defined (Request/Response enums)
- IoHelperClient with async API (spawn, read_proc_io, read_proc_io_batch, shutdown)
- Anonymous pipe-based communication
- PR_SET_PDEATHSIG for orphan prevention

### Work Items

#### 6.1 Extend ResourceMetrics Structure

**File**: `src/play_launch/src/resource_monitor.rs`

**Changes**:
```rust
pub struct ResourceMetrics {
    // ... existing fields (timestamp, cpu_usage, memory_rss, etc.) ...

    // Current I/O fields (2):
    pub total_read_bytes: u64,   // rchar
    pub total_write_bytes: u64,  // wchar

    // NEW: Extended I/O fields (5 additional):
    pub io_syscr: u64,                    // Read syscalls count
    pub io_syscw: u64,                    // Write syscalls count
    pub io_read_bytes: u64,               // Actual bytes read from storage
    pub io_write_bytes: u64,              // Actual bytes written to storage
    pub io_cancelled_write_bytes: u64,    // Write bytes later truncated
}
```

**Rationale**: Helper provides all 7 fields from `/proc/[pid]/io`. Capturing complete data enables:
- Syscall vs actual I/O analysis
- Cache hit ratio calculation (rchar/wchar vs read_bytes/write_bytes)
- Write cancellation tracking

**CSV Impact**: Adds 5 new columns to `metrics.csv` (backward compatible, existing columns unchanged)

#### 6.2 Add Helper Client to ResourceMonitor

**File**: `src/play_launch/src/resource_monitor.rs` (struct definition ~line 164)

**Changes**:
```rust
pub struct ResourceMonitor {
    // ... existing fields ...

    // NEW: I/O helper infrastructure
    io_helper: Option<IoHelperClient>,         // Helper client instance
    tokio_runtime: Option<tokio::runtime::Runtime>, // Async runtime for helper calls
    io_helper_unavailable: bool,               // Track if helper failed to spawn
}
```

**Initialization** (in `ResourceMonitor::new()` ~line 226):
```rust
// Try to spawn I/O helper (non-fatal if unavailable)
let (io_helper, tokio_runtime) = match tokio::runtime::Runtime::new() {
    Ok(rt) => {
        match rt.block_on(IoHelperClient::spawn()) {
            Ok(client) => {
                info!("I/O helper spawned successfully");
                (Some(client), Some(rt))
            }
            Err(e) => {
                warn!("I/O helper unavailable: {}. Privileged processes will have zero I/O stats.", e);
                (None, None)
            }
        }
    }
    Err(e) => {
        warn!("Failed to create Tokio runtime for I/O helper: {}", e);
        (None, None)
    }
};

Self {
    // ... existing initialization ...
    io_helper,
    tokio_runtime,
    io_helper_unavailable: io_helper.is_none(),
}
```

**Cleanup** (add to monitoring thread before exit):
```rust
// Shutdown helper before thread exits
if let Some(helper) = self.io_helper.take() {
    if let Some(rt) = self.tokio_runtime.as_ref() {
        if let Err(e) = rt.block_on(helper.shutdown()) {
            warn!("Error shutting down I/O helper: {}", e);
        }
    }
}
```

#### 6.3 Replace parse_proc_io() with Batch Helper Calls

**File**: `src/play_launch/src/resource_monitor.rs`

**Remove**: `parse_proc_io(&mut self, pid: u32) -> Result<(u64, u64)>` method (~line 652)

**Add**: Batch I/O reading in monitoring loop (before per-PID iteration ~line 1142):

```rust
// Batch read I/O stats for all PIDs (single IPC call)
let io_stats_map: HashMap<u32, ProcIoStats> = if let (Some(ref mut helper), Some(ref rt)) =
    (&mut self.io_helper, &self.tokio_runtime)
{
    // Collect all PIDs
    let pids: Vec<u32> = processes.keys().copied().collect();

    // Single batch request to helper
    match rt.block_on(helper.read_proc_io_batch(&pids)) {
        Ok(results) => {
            results.into_iter()
                .filter_map(|r| {
                    match r.result {
                        Ok(stats) => Some((r.pid, stats)),
                        Err(e) => {
                            debug!("I/O read failed for PID {}: {:?}", r.pid, e);
                            None
                        }
                    }
                })
                .collect()
        }
        Err(e) => {
            if !self.io_helper_unavailable {
                warn!("I/O helper batch request failed: {}. I/O stats will be zero.", e);
                self.io_helper_unavailable = true;
            }
            HashMap::new()
        }
    }
} else {
    // Helper unavailable - all I/O stats will be zero
    HashMap::new()
};
```

**Update**: `collect_metrics()` method (~line 363):

```rust
// OLD:
// let (total_read_bytes, total_write_bytes) = self.parse_proc_io(pid).unwrap_or((0, 0));

// NEW:
let io_stats = io_stats_map.get(&pid).cloned().unwrap_or_else(|| ProcIoStats {
    rchar: 0,
    wchar: 0,
    syscr: 0,
    syscw: 0,
    read_bytes: 0,
    write_bytes: 0,
    cancelled_write_bytes: 0,
});

// Populate extended metrics
metrics.total_read_bytes = io_stats.rchar;
metrics.total_write_bytes = io_stats.wchar;
metrics.io_syscr = io_stats.syscr;
metrics.io_syscw = io_stats.syscw;
metrics.io_read_bytes = io_stats.read_bytes;
metrics.io_write_bytes = io_stats.write_bytes;
metrics.io_cancelled_write_bytes = io_stats.cancelled_write_bytes;
```

**Rationale**:
- Batch processing: 1 IPC round-trip vs N round-trips (10-50x reduction for typical workloads)
- Non-fatal failures: Missing helper → zero I/O stats (monitoring continues)
- Debug logging per-PID errors: Aids troubleshooting without spamming logs

#### 6.4 Update CSV Writing

**File**: `src/play_launch/src/resource_monitor.rs` (~line 1183)

**Changes**:
- Add 5 new columns to CSV header
- Write new fields to CSV rows

**CSV Header** (example):
```csv
timestamp,cpu_usage,memory_rss,...,total_read_bytes,total_write_bytes,io_syscr,io_syscw,io_read_bytes,io_write_bytes,io_cancelled_write_bytes,...
```

**Backward Compatibility**: Existing columns unchanged, new columns appended

#### 6.5 Remove Dead Code Suppressions

**File**: `src/play_launch/src/io_helper_client.rs`

**Changes**:
- Remove all `#[allow(dead_code)]` attributes
- Remove "Future Integration" TODO comments
- Code is now actively used

#### 6.6 Add Makefile Target for Capabilities

**File**: `Makefile` (after build targets, ~line 116)

**Add**:
```makefile
.PHONY: setcap-io-helper
setcap-io-helper:
	@echo "Setting CAP_SYS_PTRACE on I/O helper daemon..."
	@if [ ! -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then \
		echo "Error: play_launch_io_helper not found. Run 'make build_play_launch' first."; \
		exit 1; \
	fi
	sudo setcap cap_sys_ptrace+ep install/play_launch/lib/play_launch/play_launch_io_helper
	@echo "Verifying capability:"
	@getcap install/play_launch/lib/play_launch/play_launch_io_helper
	@echo "✓ I/O helper ready for privileged process monitoring"

.PHONY: verify-io-helper
verify-io-helper:
	@echo "Checking I/O helper status..."
	@if [ -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then \
		echo "✓ Binary exists"; \
		getcap install/play_launch/lib/play_launch/play_launch_io_helper || echo "⚠ CAP_SYS_PTRACE not set (run 'make setcap-io-helper')"; \
	else \
		echo "✗ Binary not found (run 'make build_play_launch')"; \
	fi
```

**Update help text**:
```makefile
help:
	@echo "..."
	@echo "I/O Monitoring:"
	@echo "  make setcap-io-helper       - Apply CAP_SYS_PTRACE to I/O helper (requires sudo)"
	@echo "  make verify-io-helper       - Check I/O helper binary and capabilities"
```

**Note**: Capabilities must be reapplied after every rebuild (file modification clears capabilities)

#### 6.7 Update Documentation

**File**: `CLAUDE.md`

**Changes**:

1. **Build & Usage section** (~line 17):
```markdown
# Build workspace
make build

# Enable I/O monitoring for privileged processes (containers with capabilities)
make setcap-io-helper    # Requires sudo, reapply after rebuild

# Verify I/O helper status
make verify-io-helper
```

2. **Platform Notes section** (~line 300):
```markdown
### I/O Monitoring

**Standard processes**: Direct `/proc/[pid]/io` reads work without special permissions

**Privileged processes** (containers, capabilities-enabled binaries): Require helper daemon
- Helper binary: `play_launch_io_helper` (built with main package)
- Capability required: `CAP_SYS_PTRACE` (set via `make setcap-io-helper`)
- Architecture: Anonymous pipes for IPC, PR_SET_PDEATHSIG for cleanup
- Batch processing: Single IPC call per monitoring interval (efficient)

**Without helper/capabilities**:
- Warning logged once: "I/O helper unavailable. Privileged processes will have zero I/O stats."
- I/O fields show zeros for affected processes
- Monitoring continues normally for other metrics

**Extended I/O metrics** (7 fields from `/proc/[pid]/io`):
- `rchar` / `wchar` - Total bytes read/written (includes cache)
- `read_bytes` / `write_bytes` - Actual storage I/O (excludes cache)
- `syscr` / `syscw` - Read/write syscall counts
- `cancelled_write_bytes` - Writes later truncated

**Reapply after rebuild**: File capabilities are cleared when binaries change
```

3. **Troubleshooting section** (new):
```markdown
### I/O Monitoring Troubleshooting

**"I/O helper unavailable" warning**:
1. Check binary exists: `ls install/play_launch/lib/play_launch/play_launch_io_helper`
2. If missing: `make build_play_launch`
3. Set capability: `make setcap-io-helper`
4. Verify: `make verify-io-helper`

**Zero I/O stats for containers**:
- Likely missing CAP_SYS_PTRACE on helper
- Run: `make setcap-io-helper`

**"Permission denied" in helper logs**:
- Check helper capability: `getcap install/play_launch/lib/play_launch/play_launch_io_helper`
- Should show: `cap_sys_ptrace+ep`
```

### Testing Plan

#### Unit Tests
- [ ] IoHelperClient spawns successfully with valid binary
- [ ] IoHelperClient handles missing binary gracefully (returns error)
- [ ] IoHelperClient handles missing capability (warning logged)
- [ ] Batch request with empty PID list returns empty results
- [ ] Batch request with invalid PIDs returns errors per-PID
- [ ] Helper cleanup on ResourceMonitor drop/thread exit

#### Integration Tests
- [ ] **Without setcap**:
  - Build succeeds
  - Warning logged about helper unavailable
  - Monitoring continues, I/O stats all zeros
  - No crashes or permission errors

- [ ] **With setcap**:
  - Helper spawns successfully
  - I/O stats populated for regular processes
  - I/O stats populated for privileged processes (containers)
  - Batch processing: 1 helper call per monitoring interval
  - All 7 I/O fields appear in CSV

- [ ] **Helper crash mid-run**:
  - Warning logged
  - Falls back to zeros
  - Monitoring continues

- [ ] **Autoware test** (`make start-sim`):
  - All containers monitored (15 containers in test)
  - I/O stats present in all `node/*/metrics.csv` files
  - Extended fields (syscr, syscw, etc.) populated
  - No performance degradation vs direct reads

#### Performance Tests
- [ ] Batch processing vs single calls (expect 10-50x fewer IPC round-trips)
- [ ] Overhead of Tokio runtime in monitoring thread (should be negligible)
- [ ] Helper memory usage (should be minimal, <10MB)
- [ ] Helper CPU usage (should be <1% during monitoring)

#### CSV Validation
- [ ] 5 new columns added to metrics.csv
- [ ] Existing columns unchanged (backward compatible)
- [ ] All I/O fields populated correctly
- [ ] Plotting scripts handle new columns (or ignore gracefully)

### Success Criteria
- [ ] Helper binary built and installed by `make build_play_launch`
- [ ] `make setcap-io-helper` target works
- [ ] `make verify-io-helper` shows capability status
- [ ] Helper spawns automatically with ResourceMonitor
- [ ] Batch I/O reading functional (1 IPC call per interval)
- [ ] All 7 I/O fields captured in CSV
- [ ] Autoware test passes with I/O monitoring for all containers
- [ ] Documentation updated (CLAUDE.md, inline comments)
- [ ] Graceful degradation without helper/capabilities

### Dependencies
- ✅ IoHelperClient implementation (Phase 5.5 - complete)
- ✅ Helper daemon binary (Phase 5.5 - complete)
- ✅ IPC protocol (Phase 5.5 - complete)
- ⏳ ResourceMonitor integration (this phase)
- ⏳ Makefile targets (this phase)
- ⏳ Documentation updates (this phase)

### Risks and Mitigation
**Risk**: Helper overhead impacts monitoring performance
- **Mitigation**: Batch processing (already implemented in client)
- **Validation**: Benchmark before/after

**Risk**: Users forget to run `setcap` after rebuild
- **Mitigation**:
  - Clear warning when helper lacks capability
  - `make verify-io-helper` for easy checking
  - Documentation emphasizes reapplication requirement

**Risk**: Extended CSV breaks existing plotting tools
- **Mitigation**:
  - New columns appended (existing columns unchanged)
  - Test plotting scripts after integration
  - Update plotting tools if needed (future work)

**Risk**: Helper process orphaned on crash
- **Mitigation**:
  - PR_SET_PDEATHSIG (kernel-level guarantee)
  - `kill_on_drop(true)` in IoHelperClient
  - Manual testing of crash scenarios

---

## Future Considerations

### Potential Additional Subcommands
- `play_launch record` - Alias for `dump` (more intuitive naming)
- `play_launch show` - Display record.json contents in human-readable format
- `play_launch diff` - Compare two record.json files
- `play_launch validate` - Validate record.json structure without replay

### Integration Opportunities
- ROS 2 launch testing framework
- CI/CD pipelines
- Automated regression testing
- Performance benchmarking workflows
