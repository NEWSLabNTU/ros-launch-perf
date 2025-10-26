# play_launch Distribution Roadmap

## Overview

This document outlines the phased implementation of the unified `play_launch` CLI interface that provides a seamless user experience similar to `ros2 launch` and `ros2 run` commands.

## Goals

1. **Familiar UX**: ROS 2 users can use `play_launch` like native ros2 commands
2. **Transparent workflow**: Automatic dump-and-replay in a single command
3. **Backward compatible**: Existing workflows using `play_launch` continue to work
4. **Flexible**: Support dump-only and replay-only modes for advanced users
5. **Production ready**: Robust error handling and comprehensive testing

---

## Phase 1: Core Subcommand Structure

**Goal**: Refactor `play_launch` to support subcommands while maintaining backward compatibility.

**Estimated Effort**: 2-3 days

### Work Items

#### 1.1 Refactor Options Structure (options.rs)

**File**: `src/play_launch/src/options.rs`

**Changes**:
- Convert flat `Options` struct to enum-based subcommand structure
- Define `Command` enum with variants: `Launch`, `Run`, `Dump`, `Replay`
- Extract common options into shared struct `CommonOptions`
- Use clap's `#[derive(Subcommand)]` macro

**Code Structure**:
```rust
#[derive(Parser)]
pub struct Options {
    #[command(subcommand)]
    pub command: Option<Command>,

    // Flatten common options for backward compatibility
    #[command(flatten)]
    pub common: CommonOptions,
}

#[derive(Subcommand)]
pub enum Command {
    Launch(LaunchArgs),
    Run(RunArgs),
    Dump(DumpArgs),
    Replay(ReplayArgs),
}

#[derive(Args)]
pub struct CommonOptions {
    #[arg(long, default_value = "play_log")]
    pub log_dir: PathBuf,
    // ... existing options
}

#[derive(Args)]
pub struct LaunchArgs {
    pub package_or_path: String,
    pub launch_file: Option<String>,
    pub launch_arguments: Vec<String>,
}

#[derive(Args)]
pub struct RunArgs {
    pub package: String,
    pub executable: String,
    pub args: Vec<String>,
}

#[derive(Args)]
pub struct DumpArgs {
    #[command(subcommand)]
    pub subcommand: DumpSubcommand,

    #[arg(long, short = 'o', default_value = "record.json")]
    pub output: PathBuf,

    #[arg(long)]
    pub debug: bool,
}

#[derive(Subcommand)]
pub enum DumpSubcommand {
    Launch(LaunchArgs),
    Run(RunArgs),
}

#[derive(Args)]
pub struct ReplayArgs {
    #[arg(long, default_value = "record.json")]
    pub input_file: PathBuf,
    // ... existing replay options
}
```

**Testing**:
- [ ] Parse `play_launch launch pkg file.py` correctly
- [ ] Parse `play_launch run pkg exec` correctly
- [ ] Parse `play_launch dump launch pkg file.py --output out.json` correctly
- [ ] Parse `play_launch replay` correctly
- [ ] Parse `play_launch` (no subcommand) as replay for backward compatibility
- [ ] All existing options still work

#### 1.2 Update Main Entry Point (main.rs)

**File**: `src/play_launch/src/main.rs`

**Changes**:
- Match on `Options.command`
- Handle `None` variant as `Replay` for backward compatibility
- Create stub handlers for each subcommand
- Preserve existing replay logic for `Replay` variant

**Code Structure**:
```rust
#[tokio::main]
async fn main() -> eyre::Result<()> {
    let options = Options::parse();

    match options.command {
        Some(Command::Launch(args)) => {
            handle_launch(args, &options.common).await?;
        }
        Some(Command::Run(args)) => {
            handle_run(args, &options.common).await?;
        }
        Some(Command::Dump(args)) => {
            handle_dump(args).await?;
        }
        Some(Command::Replay(args)) | None => {
            // Existing replay logic
            handle_replay(args.unwrap_or_default(), &options.common).await?;
        }
    }

    Ok(())
}
```

**Testing**:
- [ ] `play_launch` (no args) invokes replay with default options
- [ ] `play_launch replay` works identically to current behavior
- [ ] All subcommands route to correct handlers
- [ ] Error messages are clear for invalid subcommands

#### 1.3 Update Dependencies (Cargo.toml)

**File**: `src/play_launch/Cargo.toml`

**Changes**:
- Ensure clap version supports derive macros (should already be present)
- Add any additional dependencies needed for subprocess management

**Testing**:
- [ ] `cargo build` succeeds
- [ ] No dependency conflicts

---

## Phase 2: dump_launch Integration

**Goal**: Implement automatic invocation of `dump_launch` for `launch` and `run` subcommands.

**Estimated Effort**: 2-3 days

### Work Items

#### 2.1 Create dump_launch Invocation Module

**File**: `src/play_launch/src/dump_launcher.rs` (NEW)

**Responsibilities**:
- Locate `dump_launch` binary via PATH
- Build command line arguments
- Execute subprocess and capture output
- Parse dump_launch output and errors
- Handle temporary record files

**Key Functions**:
```rust
pub struct DumpLauncher {
    dump_launch_path: PathBuf,
}

impl DumpLauncher {
    /// Find dump_launch in PATH
    pub fn new() -> eyre::Result<Self> {
        let path = which::which("dump_launch")
            .wrap_err("dump_launch not found in PATH. Ensure the workspace is sourced.")?;
        Ok(Self { dump_launch_path: path })
    }

    /// Execute dump_launch for a launch file
    pub async fn dump_launch(
        &self,
        package_or_path: &str,
        launch_file: Option<&str>,
        args: &[String],
        output: &Path,
        debug: bool,
    ) -> eyre::Result<()> {
        // Build command
        // Execute subprocess
        // Wait for completion
        // Handle errors
    }

    /// Execute dump_launch for a single node
    pub async fn dump_run(
        &self,
        package: &str,
        executable: &str,
        args: &[String],
        output: &Path,
        debug: bool,
    ) -> eyre::Result<()> {
        // Similar to dump_launch but for run mode
    }
}
```

**Dependencies**:
- Add `which` crate to find binaries in PATH
- Use tokio's `Command` for async subprocess execution

**Testing**:
- [ ] `DumpLauncher::new()` finds dump_launch when workspace is sourced
- [ ] Returns clear error when dump_launch not found
- [ ] `dump_launch()` correctly invokes dump_launch with package name
- [ ] `dump_launch()` correctly invokes dump_launch with file path
- [ ] `dump_run()` correctly invokes dump_launch for single nodes
- [ ] Output file is created at specified path
- [ ] Errors from dump_launch are properly reported
- [ ] Debug flag is passed through correctly

#### 2.2 Implement Launch Subcommand Handler

**File**: `src/play_launch/src/main.rs`

**Changes**:
- Implement `handle_launch()` function
- Call dump_launcher to generate record.json
- Call existing replay logic with generated record

**Code Structure**:
```rust
async fn handle_launch(args: LaunchArgs, common: &CommonOptions) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()?;

    // Determine output path (temp or user-specified)
    let record_path = common.input_file.clone(); // or create temp file

    // Execute dump
    info!("Recording launch execution...");
    launcher.dump_launch(
        &args.package_or_path,
        args.launch_file.as_deref(),
        &args.launch_arguments,
        &record_path,
        false, // debug flag
    ).await?;

    // Execute replay with recorded file
    info!("Replaying launch execution...");
    let replay_args = ReplayArgs {
        input_file: record_path,
        ..Default::default()
    };
    handle_replay(replay_args, common).await?;

    Ok(())
}
```

**Testing**:
- [ ] `play_launch launch demo_nodes_cpp talker_listener.launch.py` works end-to-end
- [ ] Launch arguments are properly passed to dump_launch
- [ ] record.json is generated and contains expected data
- [ ] Replay executes successfully after dump
- [ ] Errors during dump are reported clearly
- [ ] Errors during replay are reported clearly
- [ ] Logs are saved to correct directory

#### 2.3 Implement Run Subcommand Handler

**File**: `src/play_launch/src/main.rs`

**Changes**:
- Implement `handle_run()` function
- Similar to `handle_launch()` but for single nodes

**Code Structure**:
```rust
async fn handle_run(args: RunArgs, common: &CommonOptions) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()?;

    let record_path = common.input_file.clone();

    info!("Recording node execution...");
    launcher.dump_run(
        &args.package,
        &args.executable,
        &args.args,
        &record_path,
        false,
    ).await?;

    info!("Replaying node execution...");
    let replay_args = ReplayArgs {
        input_file: record_path,
        ..Default::default()
    };
    handle_replay(replay_args, common).await?;

    Ok(())
}
```

**Testing**:
- [ ] `play_launch run demo_nodes_cpp talker` works end-to-end
- [ ] Node arguments are properly passed
- [ ] Single node is recorded and replayed correctly

#### 2.4 Implement Dump Subcommand Handler

**File**: `src/play_launch/src/main.rs`

**Changes**:
- Implement `handle_dump()` function
- Execute dump_launch without replay

**Code Structure**:
```rust
async fn handle_dump(args: DumpArgs) -> eyre::Result<()> {
    let launcher = DumpLauncher::new()?;

    match args.subcommand {
        DumpSubcommand::Launch(launch_args) => {
            launcher.dump_launch(
                &launch_args.package_or_path,
                launch_args.launch_file.as_deref(),
                &launch_args.launch_arguments,
                &args.output,
                args.debug,
            ).await?;
        }
        DumpSubcommand::Run(run_args) => {
            launcher.dump_run(
                &run_args.package,
                &run_args.executable,
                &run_args.args,
                &args.output,
                args.debug,
            ).await?;
        }
    }

    info!("Dump completed: {}", args.output.display());
    Ok(())
}
```

**Testing**:
- [ ] `play_launch dump launch demo_nodes_cpp talker_listener.launch.py --output test.json` creates test.json
- [ ] No replay is executed
- [ ] Debug flag works correctly
- [ ] Custom output path is respected

#### 2.5 Update Cargo.toml

**File**: `src/play_launch/Cargo.toml`

**Changes**:
```toml
[dependencies]
# ... existing dependencies
which = "6.0"  # For finding binaries in PATH
```

**Testing**:
- [ ] `cargo build` succeeds
- [ ] `which` crate correctly finds dump_launch

---

## Phase 3: Documentation & Polish

**Goal**: Complete documentation and user-facing polish.

**Estimated Effort**: 1 day

### Work Items

#### 3.1 Update CLAUDE.md

**File**: `CLAUDE.md`

**Changes**:
- Update "Running the Tools" section with new CLI
- Add subcommand examples
- Document the automatic dump-and-replay workflow
- Update Architecture section with dump_launcher module

**Testing**:
- [ ] Documentation is accurate and complete
- [ ] Examples in CLAUDE.md work as documented

#### 3.2 Create CLI Interface Documentation

**File**: `docs/cli-interface.md`

**Status**: ✅ Already created in this session

**Verify**:
- [ ] All commands documented
- [ ] All options documented
- [ ] Examples are accurate
- [ ] Troubleshooting section is helpful

#### 3.3 Update README (if exists)

**File**: `README.md`

**Changes**:
- Update quick start examples
- Show new CLI interface
- Update installation instructions

#### 3.4 Add Help Text

**File**: `src/play_launch/src/options.rs`

**Changes**:
- Add comprehensive help text to all commands and options
- Add examples to help output

**Example**:
```rust
#[derive(Parser)]
#[command(name = "play_launch")]
#[command(about = "Record and replay ROS 2 launches with inspection capabilities")]
#[command(after_help = "Examples:\n  \
    play_launch launch demo_nodes_cpp talker_listener.launch.py\n  \
    play_launch run demo_nodes_cpp talker\n  \
    play_launch dump launch autoware_launch planning_simulator.launch.xml --output autoware.json")]
pub struct Options {
    // ...
}
```

**Testing**:
- [ ] `play_launch --help` shows comprehensive help
- [ ] `play_launch launch --help` shows launch-specific help
- [ ] `play_launch run --help` shows run-specific help
- [ ] `play_launch dump --help` shows dump-specific help
- [ ] Help text is clear and accurate

---

## Phase 4: Testing & Validation

**Goal**: Comprehensive testing across multiple scenarios and edge cases.

**Estimated Effort**: 2-3 days

### Work Items

#### 4.1 Unit Tests

**Files**:
- `src/play_launch/src/dump_launcher.rs`
- `src/play_launch/src/options.rs`

**Test Cases**:
- [ ] Option parsing for all subcommands
- [ ] dump_launch binary discovery
- [ ] Command line argument construction
- [ ] Error handling for missing dump_launch
- [ ] Error handling for invalid arguments

#### 4.2 Integration Tests with demo_nodes_cpp

**Test Scenarios**:
- [ ] `play_launch launch demo_nodes_cpp talker_listener.launch.py`
  - Verify nodes start correctly
  - Verify communication between talker and listener
  - Check log output in play_log/
- [ ] `play_launch run demo_nodes_cpp talker`
  - Single node execution
  - Verify node outputs
- [ ] `play_launch dump launch demo_nodes_cpp talker_listener.launch.py --output test.json`
  - Verify test.json is created
  - Verify JSON structure is valid
  - No replay execution
- [ ] `play_launch replay --input-file test.json`
  - Replay from previously dumped file
  - Verify same behavior as original launch

#### 4.3 Integration Tests with Autoware

**Test Scenarios**:
- [ ] `play_launch launch autoware_launch planning_simulator.launch.xml map_path:=...`
  - All 15 containers start
  - All 54 composable nodes load successfully
  - No orphan nodes warnings
  - Autonomous driving test passes
- [ ] With monitoring enabled: `--enable-monitoring --monitor-interval-ms 1000`
  - metrics.csv files created for all nodes
  - Resource data is valid
- [ ] With process control: `--config config.yaml`
  - CPU affinity applied correctly
  - Nice values set correctly
- [ ] With service readiness: `--wait-for-service-ready --service-ready-timeout-secs 300`
  - All container services become ready
  - Composable nodes load successfully

#### 4.4 Error Handling Tests

**Test Scenarios**:
- [ ] dump_launch not in PATH
  - Error message: "dump_launch not found in PATH. Ensure the workspace is sourced."
- [ ] Invalid package name
  - Error from dump_launch is displayed
- [ ] Invalid launch file
  - Error from dump_launch is displayed
- [ ] Launch file execution fails
  - Dump phase fails with clear error
  - Replay is not attempted
- [ ] record.json is malformed
  - Replay phase fails with parsing error
- [ ] Ctrl-C during dump
  - Cleanup happens correctly
  - Partial record.json is not used
- [ ] Ctrl-C during replay
  - All processes are killed
  - CleanupGuard works correctly

#### 4.5 Backward Compatibility Tests

**Test Scenarios**:
- [ ] `play_launch` (no subcommand, current usage)
  - Works exactly as before
  - Uses default record.json
  - All options work
- [ ] `play_launch replay`
  - Explicitly specifying replay subcommand
  - Identical to no-subcommand usage
- [ ] Existing scripts using `ros2 run dump_launch dump_launch ...` continue to work
- [ ] Existing scripts using `ros2 run play_launch play_launch ...` continue to work

#### 4.6 Edge Cases

**Test Scenarios**:
- [ ] Launch file path with spaces
- [ ] Very long argument lists
- [ ] Special characters in arguments
- [ ] Multiple `:=` in single argument
- [ ] Empty package name or launch file
- [ ] Relative paths vs absolute paths
- [ ] Symlinked launch files
- [ ] Launch files in current directory

---

## Phase 5: Optional Enhancements

**Goal**: Nice-to-have features that improve user experience.

**Estimated Effort**: 1-2 days (as time permits)

### Work Items

#### 5.1 Environment Variables

**Implementation**:
- `PLAY_LAUNCH_RECORD_FILE`: Default record file path
- `PLAY_LAUNCH_LOG_DIR`: Default log directory
- `PLAY_LAUNCH_CONFIG`: Default config file

**File**: `src/play_launch/src/options.rs`

**Testing**:
- [ ] Environment variables are respected
- [ ] CLI arguments override environment variables

#### 5.2 Bash/Zsh Completion

**Files**:
- `src/play_launch/completions/play_launch.bash` (NEW)
- `src/play_launch/completions/play_launch.zsh` (NEW)

**Implementation**:
- Use clap's completion generation
- Install completion scripts during package build

**Testing**:
- [ ] Tab completion works for subcommands
- [ ] Tab completion works for options
- [ ] Package name completion (if feasible)

#### 5.3 Progress Indicators

**Implementation**:
- Show progress during dump phase
- Show progress during composable node loading
- Use indicatif crate for progress bars

**Testing**:
- [ ] Progress is displayed correctly
- [ ] Works in non-TTY environments (CI)

#### 5.4 Colored Output

**Implementation**:
- Use colored crate for better readability
- Color-code info, warning, error messages

**Testing**:
- [ ] Colors work in terminal
- [ ] Colors disabled in non-TTY

---

## Success Criteria

### Must Have (Phases 1-4)
- [ ] All subcommands work correctly
- [ ] Backward compatibility maintained
- [ ] dump_launch integration functional
- [ ] Error handling is robust
- [ ] Documentation is complete
- [ ] Autoware test passes
- [ ] demo_nodes_cpp tests pass

### Nice to Have (Phase 5)
- [ ] Environment variable support
- [ ] Completion scripts
- [ ] Progress indicators
- [ ] Colored output

---

## Dependencies

### External
- `dump_launch` must be installed and in PATH
- ROS 2 workspace must be sourced

### Internal
- clap crate (already present)
- which crate (to be added)
- tokio (already present)

---

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Breaking backward compatibility | High | Extensive testing, keep no-subcommand mode as replay |
| dump_launch changes breaking integration | Medium | Version check, clear error messages |
| Subprocess error handling complexity | Medium | Comprehensive error handling, user-friendly messages |
| Performance overhead from subprocess | Low | Dump is one-time cost, replay is same performance |

---

## Timeline Estimate

| Phase | Days | Dependencies |
|-------|------|--------------|
| Phase 1 | 2-3 | None |
| Phase 2 | 2-3 | Phase 1 complete |
| Phase 3 | 1 | Phase 2 complete |
| Phase 4 | 2-3 | Phases 1-3 complete |
| **Total (MVP)** | **7-10 days** | |
| Phase 5 (optional) | 1-2 | MVP complete |

---

## Maintenance Considerations

### After Implementation
- Keep CLI interface stable (avoid breaking changes)
- Document any new options thoroughly
- Maintain backward compatibility
- Update help text when adding features

### Future Enhancements
- Consider `play_launch record` as alias for `dump`
- Consider `play_launch show` to display record.json contents
- Consider `play_launch diff` to compare two record.json files
- Integration with ROS 2 launch testing framework
