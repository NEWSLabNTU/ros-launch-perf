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
