# Resource Monitoring and Process Control Design

## Overview

This document describes the design for adding resource monitoring and process control features to play_launch. The goal is to enable detailed per-node performance analysis and allow fine-grained control over process scheduling.

## Motivation

- **Performance Analysis**: Understanding resource usage per node helps identify bottlenecks and optimize system performance
- **Reproducible Testing**: Controlling CPU affinity and nice values enables consistent performance measurements
- **Live Monitoring**: CSV logging enables real-time visualization and post-analysis
- **Debugging**: Resource metrics help diagnose memory leaks, CPU spikes, and I/O bottlenecks

## Requirements

### Functional Requirements

1. **Resource Metrics Collection**
   - CPU usage (percentage, user time, system time)
   - Memory usage (RSS, VMS, shared memory)
   - I/O statistics (read bytes, write bytes, read/write operations)
   - Process state (running, sleeping, zombie, etc.)
   - Thread count
   - File descriptor count

2. **Data Persistence**
   - CSV format for easy analysis
   - Timestamped entries
   - Per-node logging
   - Configurable sampling interval

3. **Process Control**
   - Set CPU affinity per node
   - Set nice value per node
   - Configuration file support

4. **Live Monitoring (Optional)**
   - Real-time CSV updates
   - Compatible with external visualization tools

### Non-Functional Requirements

- Low overhead (monitoring should not significantly impact system performance)
- Minimal memory footprint
- Graceful degradation if monitoring fails
- No impact on node execution if monitoring is disabled

## Design

### Architecture

```
play_launch
├── main.rs (spawn monitoring thread)
├── config.rs (NEW)
│   ├── struct RuntimeConfig
│   ├── struct MonitoringSettings
│   ├── struct ProcessConfig
│   ├── fn load_runtime_config()
│   ├── fn apply_affinity()
│   └── fn apply_nice()
├── resource_monitor.rs (NEW)
│   ├── struct ResourceMonitor
│   ├── fn spawn_monitor_thread()
│   ├── fn collect_metrics()
│   └── fn write_csv()
└── execution.rs (MODIFIED)
    └── fn spawn_nodes() - apply process config
```

### Data Structures

#### RuntimeConfig (config.rs)

```rust
/// Runtime configuration for play_launch
#[derive(Debug, Clone, Deserialize, Default)]
pub struct RuntimeConfig {
    /// Resource monitoring settings
    #[serde(default)]
    pub monitoring: MonitoringSettings,

    /// Per-process configurations
    #[serde(default)]
    pub processes: Vec<ProcessConfig>,

    // Future: execution settings, logging settings, etc.
}

/// Global monitoring settings
#[derive(Debug, Clone, Deserialize)]
pub struct MonitoringSettings {
    /// Enable monitoring (default: false, overridden by --enable-monitoring flag)
    #[serde(default)]
    pub enabled: bool,

    /// Sampling interval in milliseconds (default: 1000)
    #[serde(default = "default_sample_interval")]
    pub sample_interval_ms: u64,

    /// Monitor all nodes by default (default: true when monitoring is enabled)
    #[serde(default = "default_true")]
    pub monitor_all_nodes: bool,

    /// Only monitor nodes matching these patterns (empty = all nodes)
    #[serde(default)]
    pub monitor_patterns: Vec<String>,
}

fn default_sample_interval() -> u64 { 1000 }
fn default_true() -> bool { true }

/// Configuration for individual process control
#[derive(Debug, Clone, Deserialize)]
pub struct ProcessConfig {
    /// Node name or pattern (supports glob patterns)
    pub node_pattern: String,

    /// Enable monitoring for this node (default: inherit from global)
    #[serde(default)]
    pub monitor: Option<bool>,

    /// CPU cores to pin this process to (e.g., [0, 1, 2])
    #[serde(default)]
    pub cpu_affinity: Vec<usize>,

    /// Nice value (-20 to 19, lower = higher priority)
    #[serde(default)]
    pub nice: Option<i32>,
}

impl ProcessConfig {
    /// Check if this config matches a node name
    pub fn matches(&self, node_name: &str) -> bool {
        glob::Pattern::new(&self.node_pattern)
            .map(|pattern| pattern.matches(node_name))
            .unwrap_or(false)
    }

    /// Check if monitoring is enabled for this node
    pub fn should_monitor(&self, global_enabled: bool) -> bool {
        self.monitor.unwrap_or(global_enabled)
    }
}
```

Example configuration file (`config.yaml`):

```yaml
# Resource monitoring settings
monitoring:
  enabled: true              # Can be overridden by --enable-monitoring flag
  sample_interval_ms: 1000   # Sample every 1 second
  monitor_all_nodes: true    # Monitor all nodes by default
  # monitor_patterns:        # Optional: only monitor matching patterns
  #   - "*/planning/*"
  #   - "*/control/*"

# Per-process configurations
processes:
  # High-priority planning nodes with monitoring
  - node_pattern: "*/behavior_path_planner"
    monitor: true
    cpu_affinity: [0, 1]
    nice: -10

  # Control nodes on dedicated cores
  - node_pattern: "*/control/*"
    monitor: true
    cpu_affinity: [2, 3]
    nice: -5

  # Low priority visualization - disable monitoring
  - node_pattern: "*/rviz*"
    monitor: false           # Don't monitor even if global monitoring is on
    cpu_affinity: [4, 5, 6, 7]
    nice: 10

  # Perception nodes with monitoring but no process control
  - node_pattern: "*/perception/**"
    monitor: true

  # Apply nice value without monitoring
  - node_pattern: "*/localization/*"
    nice: -5

  # Just monitor, no process control
  - node_pattern: "*/system/component_state_monitor"
    monitor: true
```

Minimal monitoring example (`config_monitoring.yaml`):

```yaml
# Enable monitoring for all nodes with default settings
monitoring:
  enabled: true
```

Process control only example (`config_process_control.yaml`):

```yaml
# No monitoring, just process control
monitoring:
  enabled: false

processes:
  - node_pattern: "*/planning/*"
    nice: -10
    cpu_affinity: [0, 1, 2, 3]

  - node_pattern: "*/control/*"
    nice: -5
    cpu_affinity: [4, 5]
```

#### ResourceMetrics (resource_monitor.rs)

```rust
/// Resource metrics for a single process at a point in time
#[derive(Debug, Clone)]
pub struct ResourceMetrics {
    pub timestamp: SystemTime,
    pub pid: u32,
    pub node_name: String,

    // CPU metrics
    pub cpu_percent: f64,
    pub cpu_user_time: u64,    // microseconds
    pub cpu_system_time: u64,  // microseconds

    // Memory metrics
    pub rss_bytes: u64,        // Resident Set Size
    pub vms_bytes: u64,        // Virtual Memory Size
    pub shared_bytes: u64,     // Shared memory

    // I/O metrics
    pub io_read_bytes: u64,
    pub io_write_bytes: u64,
    pub io_read_ops: u64,
    pub io_write_ops: u64,

    // Process info
    pub state: ProcessState,
    pub num_threads: u32,
    pub num_fds: u32,          // File descriptors
}

#[derive(Debug, Clone, Copy)]
pub enum ProcessState {
    Running,
    Sleeping,
    Waiting,
    Zombie,
    Stopped,
    Unknown,
}

/// Configuration for resource monitoring
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub enabled: bool,
    pub sample_interval_ms: u64,
    pub log_dir: PathBuf,
}
```

### CSV File Format

Each node gets its own CSV file: `play_log/<timestamp>/metrics/<namespace>/<node_name>.csv`

```csv
timestamp,pid,cpu_percent,cpu_user_us,cpu_system_us,rss_bytes,vms_bytes,shared_bytes,io_read_bytes,io_write_bytes,io_read_ops,io_write_ops,state,num_threads,num_fds
2025-10-15T08:00:00.000Z,12345,15.3,123456,78910,104857600,524288000,8388608,1048576,524288,100,50,Running,8,42
2025-10-15T08:00:01.000Z,12345,16.1,135678,81234,104923136,524288000,8388608,1049600,525312,102,51,Running,8,42
```

### Implementation Details

#### 1. Resource Metric Collection

Use the **sysinfo** crate for cross-platform process monitoring:

```rust
use sysinfo::{System, ProcessExt, Pid};

pub struct ResourceMonitor {
    system: System,
    prev_cpu_times: HashMap<u32, (u64, u64)>,  // pid -> (user_time, system_time)
}

impl ResourceMonitor {
    pub fn new() -> Self {
        Self {
            system: System::new_all(),
            prev_cpu_times: HashMap::new(),
        }
    }

    fn collect_metrics(&mut self, pid: u32, node_name: &str) -> Result<ResourceMetrics> {
        // Refresh process information
        self.system.refresh_process(Pid::from_u32(pid));

        let process = self.system.process(Pid::from_u32(pid))
            .ok_or_else(|| eyre::eyre!("Process {} not found", pid))?;

        // Get disk usage (cumulative read/write bytes)
        let disk_usage = process.disk_usage();

        // Count open file descriptors (Linux-specific via sysinfo)
        #[cfg(target_os = "linux")]
        let num_fds = std::fs::read_dir(format!("/proc/{}/fd", pid))
            .map(|entries| entries.count() as u32)
            .unwrap_or(0);
        #[cfg(not(target_os = "linux"))]
        let num_fds = 0;

        // Map sysinfo process status to our ProcessState enum
        let state = match process.status() {
            sysinfo::ProcessStatus::Run => ProcessState::Running,
            sysinfo::ProcessStatus::Sleep => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Idle => ProcessState::Sleeping,
            sysinfo::ProcessStatus::Zombie => ProcessState::Zombie,
            sysinfo::ProcessStatus::Stop => ProcessState::Stopped,
            _ => ProcessState::Unknown,
        };

        Ok(ResourceMetrics {
            timestamp: SystemTime::now(),
            pid,
            node_name: node_name.to_string(),
            cpu_percent: process.cpu_usage() as f64,
            cpu_user_time: process.run_time(),  // Total accumulated runtime
            cpu_system_time: 0,  // sysinfo doesn't separate user/system time
            rss_bytes: process.memory(),
            vms_bytes: process.virtual_memory(),
            shared_bytes: 0,  // Not available via sysinfo
            io_read_bytes: disk_usage.total_read_bytes,
            io_write_bytes: disk_usage.total_written_bytes,
            io_read_ops: 0,  // Not available via sysinfo
            io_write_ops: 0,  // Not available via sysinfo
            state,
            num_threads: process.tasks().map(|t| t.len() as u32).unwrap_or(0),
            num_fds,
        })
    }
}
```

**Note:** The sysinfo crate provides most metrics but has some limitations:
- No separate user/system CPU time (only total)
- No I/O operation counts (only bytes)
- No shared memory size

These missing metrics can be optionally enhanced with direct `/proc` reads on Linux if needed.

#### 2. Monitoring Thread

Background thread that periodically samples all tracked processes:

```rust
pub fn spawn_monitor_thread(
    config: MonitorConfig,
    process_registry: Arc<Mutex<HashMap<u32, String>>>,  // pid -> node_name
) -> JoinHandle<()> {
    thread::spawn(move || {
        let mut monitor = ResourceMonitor::new();
        let interval = Duration::from_millis(config.sample_interval_ms);

        loop {
            // Refresh all processes in one call for efficiency
            monitor.system.refresh_processes();

            let processes = process_registry.lock().unwrap().clone();

            for (pid, node_name) in processes {
                match monitor.collect_metrics(pid, &node_name) {
                    Ok(metrics) => {
                        if let Err(e) = write_csv(&config.log_dir, &node_name, &metrics) {
                            warn!("Failed to write metrics for {}: {}", node_name, e);
                        }
                    }
                    Err(e) => {
                        debug!("Failed to collect metrics for {} (pid {}): {}", node_name, pid, e);
                        // Process may have exited, remove from registry
                    }
                }
            }

            thread::sleep(interval);
        }
    })
}
```

**Note:** sysinfo's `System::refresh_processes()` is more efficient than individual refreshes when monitoring many processes.

#### 3. Process Control Application

Apply CPU affinity and nice values after spawning:

```rust
impl ProcessConfig {
    pub fn apply(&self, pid: u32) -> Result<()> {
        // Apply CPU affinity
        if !self.cpu_affinity.is_empty() {
            self.set_affinity(pid)?;
        }

        // Apply nice value
        if let Some(nice) = self.nice {
            self.set_nice(pid, nice)?;
        }

        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn set_affinity(&self, pid: u32) -> Result<()> {
        use libc::{cpu_set_t, sched_setaffinity, CPU_SET, CPU_ZERO};

        unsafe {
            let mut cpu_set: cpu_set_t = std::mem::zeroed();
            CPU_ZERO(&mut cpu_set);

            for &cpu in &self.cpu_affinity {
                CPU_SET(cpu, &mut cpu_set);
            }

            let ret = sched_setaffinity(
                pid as libc::pid_t,
                std::mem::size_of::<cpu_set_t>(),
                &cpu_set,
            );

            if ret != 0 {
                return Err(eyre::eyre!("Failed to set CPU affinity: {}",
                    std::io::Error::last_os_error()));
            }
        }

        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn set_nice(&self, pid: u32, nice: i32) -> Result<()> {
        use libc::{setpriority, PRIO_PROCESS};

        unsafe {
            let ret = setpriority(PRIO_PROCESS as u32, pid, nice);
            if ret != 0 {
                return Err(eyre::eyre!("Failed to set nice value: {}",
                    std::io::Error::last_os_error()));
            }
        }

        Ok(())
    }
}
```

#### 4. Configuration Loading and Resolution

```rust
/// Load and resolve runtime configuration
pub fn load_runtime_config(
    config_path: Option<&Path>,
    enable_monitoring_flag: bool,
    monitor_interval_override: Option<u64>,
) -> Result<ResolvedRuntimeConfig> {
    // Load config file or use defaults
    let mut config = if let Some(path) = config_path {
        let content = std::fs::read_to_string(path)?;
        serde_yaml::from_str::<RuntimeConfig>(&content)?
    } else {
        RuntimeConfig::default()
    };

    // CLI flag overrides config file
    if enable_monitoring_flag {
        config.monitoring.enabled = true;
    }

    // CLI interval overrides config file
    if let Some(interval) = monitor_interval_override {
        config.monitoring.sample_interval_ms = interval;
    }

    Ok(ResolvedRuntimeConfig {
        monitoring: ResolvedMonitoringConfig {
            enabled: config.monitoring.enabled,
            sample_interval_ms: config.monitoring.sample_interval_ms,
            monitor_all_nodes: config.monitoring.monitor_all_nodes,
            monitor_patterns: config.monitoring.monitor_patterns,
            process_configs: config.processes,
        },
    })
}

/// Resolved runtime configuration ready for use
pub struct ResolvedRuntimeConfig {
    pub monitoring: ResolvedMonitoringConfig,
    // Future: execution settings, logging settings, etc.
}

/// Resolved monitoring configuration
pub struct ResolvedMonitoringConfig {
    pub enabled: bool,
    pub sample_interval_ms: u64,
    pub monitor_all_nodes: bool,
    pub monitor_patterns: Vec<String>,
    pub process_configs: Vec<ProcessConfig>,
}

impl ResolvedMonitoringConfig {
    /// Check if a node should be monitored
    pub fn should_monitor(&self, node_name: &str) -> bool {
        if !self.enabled {
            return false;
        }

        // Check process-specific config first
        for config in &self.process_configs {
            if config.matches(node_name) {
                if let Some(monitor) = config.monitor {
                    return monitor;
                }
            }
        }

        // Check monitor_patterns if specified
        if !self.monitor_patterns.is_empty() {
            return self.monitor_patterns.iter().any(|pattern| {
                glob::Pattern::new(pattern)
                    .map(|p| p.matches(node_name))
                    .unwrap_or(false)
            });
        }

        // Default: monitor all nodes if enabled
        self.monitor_all_nodes
    }

    /// Get process config for a node
    pub fn get_process_config(&self, node_name: &str) -> Option<&ProcessConfig> {
        self.process_configs
            .iter()
            .find(|config| config.matches(node_name))
    }
}
```

#### 5. Integration with spawn_nodes()

Modify `execution.rs` to apply process config and register processes:

```rust
pub fn spawn_nodes(
    node_contexts: Vec<NodeContext>,
    runtime_config: &ResolvedRuntimeConfig,
    monitor_registry: &Arc<Mutex<HashMap<u32, String>>>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    let monitoring_config = &runtime_config.monitoring;
    node_contexts
        .into_iter()
        .filter_map(|context| {
            let exec = match context.to_exec_context() {
                Ok(exec) => exec,
                Err(err) => {
                    error!("Unable to prepare execution for node: {err}");
                    return None;
                }
            };

            let ExecutionContext {
                log_name,
                output_dir,
                mut command,
            } = exec;

            let mut child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}");
                    return None;
                }
            };

            // Apply process configuration and monitoring
            if let Some(pid) = child.id() {
                // Register for monitoring if enabled
                if monitoring_config.should_monitor(&log_name) {
                    monitor_registry.lock().unwrap().insert(pid, log_name.clone());
                    debug!("Registered {} (pid {}) for monitoring", log_name, pid);
                }

                // Apply CPU affinity and nice value if configured
                if let Some(config) = monitoring_config.get_process_config(&log_name) {
                    if let Err(e) = config.apply(pid) {
                        warn!("Failed to apply process config to {}: {}", log_name, e);
                    } else {
                        let mut applied = Vec::new();
                        if !config.cpu_affinity.is_empty() {
                            applied.push(format!("affinity={:?}", config.cpu_affinity));
                        }
                        if let Some(nice) = config.nice {
                            applied.push(format!("nice={}", nice));
                        }
                        if !applied.is_empty() {
                            info!("Applied to {} (pid {}): {}", log_name, pid, applied.join(", "));
                        }
                    }
                }
            }

            let task = async move {
                wait_for_node(&log_name, &output_dir, child).await
            };
            Some(task)
        })
        .collect()
}
```

### CLI Options

New options for `play_launch`:

```rust
pub struct Options {
    // ... existing fields ...

    /// Runtime configuration file
    /// Contains monitoring settings, process control, and other runtime options
    #[clap(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable resource monitoring for all nodes
    /// This overrides the 'monitoring.enabled' setting in the config file
    #[clap(long)]
    pub enable_monitoring: bool,

    /// Resource sampling interval in milliseconds (overrides config file)
    #[clap(long, value_name = "MS")]
    pub monitor_interval_ms: Option<u64>,
}
```

**Usage examples:**

```bash
# Enable monitoring for all nodes with default settings (1s interval)
play_launch --enable-monitoring

# Use config file (monitoring enabled/disabled per config)
play_launch --config config.yaml

# Short form
play_launch -c config.yaml

# Enable monitoring and override config file
play_launch --config config.yaml --enable-monitoring

# Enable monitoring with custom interval
play_launch --enable-monitoring --monitor-interval-ms 500

# Process control only (no monitoring)
play_launch --config config_process_control.yaml

# Full control: config file + override interval
play_launch --config config.yaml --monitor-interval-ms 2000
```

**Priority order (highest to lowest):**
1. CLI flags (`--enable-monitoring`, `--monitor-interval-ms`)
2. Config file settings (`monitoring.enabled`, `monitoring.sample_interval_ms`)
3. Defaults (monitoring disabled, 1000ms interval)

### Log Directory Structure

```
play_log/
└── YYYY-MM-DD_HH-MM-SS/
    ├── params_files/
    ├── node/
    │   └── <package>/<exec_name>/
    │       ├── out
    │       ├── err
    │       ├── pid
    │       ├── status
    │       └── cmdline
    ├── load_node/
    │   └── <container>/<package>/<plugin>/
    │       ├── service_response.1
    │       └── status
    └── metrics/          # NEW
        └── <namespace>/
            └── <node_name>.csv
```

## Dependencies

### Rust Crates

- **`sysinfo`** (primary) - Cross-platform system information library
  - Provides all process metrics: CPU, memory, I/O, threads, FDs
  - Actively maintained, well-tested, comprehensive API
  - Eliminates need for manual `/proc` parsing
  - Cross-platform bonus: Linux, Windows, macOS, FreeBSD, etc.
- `glob` - Pattern matching for node names
- `csv` - CSV writing
- `serde_yaml` - Configuration file parsing

## Performance Considerations

### Overhead Analysis

- **CPU**: Reading `/proc` files is relatively fast (~100-200μs per process)
- **I/O**: CSV appends are buffered and amortized
- **Memory**: Minimal (one buffer per monitored process)

### Optimization Strategies

1. **Single System Instance**: Reuse one `sysinfo::System` throughout monitoring thread
2. **Batch Refresh**: Call `system.refresh_processes()` once per cycle instead of per-process
3. **Batch Writing**: Buffer metrics and write in batches
4. **Sampling Rate**: Default 1Hz, configurable for lower overhead
5. **Lazy Initialization**: Only create CSV files when monitoring is enabled
6. **Process Filtering**: Only monitor processes matching patterns (if specified)

## Implementation Plan

### Phase 1: Core Infrastructure (Target: 1 week)

**Note:** Using sysinfo crate significantly simplifies implementation and reduces timeline.

#### Work Items

**1.1 Runtime Config Module** `config.rs`
- [ ] Define `RuntimeConfig`, `MonitoringSettings`, and `ProcessConfig` structs
- [ ] Implement YAML deserialization with serde_yaml
- [ ] Implement `load_runtime_config()` with priority resolution
- [ ] Implement `ResolvedRuntimeConfig` and `ResolvedMonitoringConfig`
- [ ] Implement `should_monitor()` logic with pattern matching
- [ ] Implement glob pattern matching for node names
- [ ] Implement `set_affinity()` using `sched_setaffinity()`
- [ ] Implement `set_nice()` using `setpriority()`
- [ ] Add error handling for permission issues
- [ ] Add validation for CPU affinity and nice values

**1.2 Resource Metrics Collection** `resource_monitor.rs`
- [ ] Define `ResourceMetrics` and `ProcessState` structs
- [ ] Initialize sysinfo `System` instance
- [ ] Implement `collect_metrics()` using sysinfo API
  - [ ] Get CPU usage via `process.cpu_usage()`
  - [ ] Get memory (RSS/VMS) via `process.memory()` / `process.virtual_memory()`
  - [ ] Get disk I/O via `process.disk_usage()`
  - [ ] Get thread count via `process.tasks()`
  - [ ] Map process status to `ProcessState` enum
- [ ] Implement FD counting via `/proc/<pid>/fd/` (Linux-only fallback)
- [ ] Handle process exit gracefully when sysinfo returns None

**1.3 CSV Logging** `resource_monitor.rs`
- [ ] Implement CSV header generation
- [ ] Implement CSV row writing with buffering
- [ ] Create metrics directory structure
- [ ] Handle concurrent writes safely
- [ ] Implement file rotation if needed

**1.4 Monitoring Thread** `resource_monitor.rs`
- [ ] Implement background thread with configurable interval
- [ ] Create single `System` instance (reused for efficiency)
- [ ] Call `system.refresh_processes()` once per sampling cycle
- [ ] Implement process registry (pid -> node_name mapping)
- [ ] Implement graceful shutdown mechanism
- [ ] Add metrics for monitoring thread itself (self-monitoring)

**1.5 Integration** `main.rs`, `execution.rs`, `options.rs`
- [ ] Add CLI flags: `--config` (short: `-c`), `--enable-monitoring`, `--monitor-interval-ms`
- [ ] Load and resolve runtime config in `main.rs`
- [ ] Modify `spawn_nodes()` to accept `ResolvedRuntimeConfig`
- [ ] Modify `spawn_nodes()` to check `should_monitor()` before registering
- [ ] Modify `spawn_nodes()` to apply process config
- [ ] Conditionally spawn monitoring thread only if enabled
- [ ] Pass process registry to monitoring thread
- [ ] Clean up monitoring thread on exit

**1.6 Dependencies**
- [ ] Add `sysinfo` crate to Cargo.toml (primary dependency)
- [ ] Add `csv` crate to Cargo.toml
- [ ] Add `serde_yaml` crate to Cargo.toml
- [ ] Add `glob` crate to Cargo.toml
- [ ] Update documentation

#### Test Cases

**TC1.1: Configuration Loading and Resolution**
- [ ] Load valid YAML configuration
- [ ] Handle missing config file (use defaults)
- [ ] Handle invalid YAML syntax gracefully
- [ ] CLI flag overrides config file setting
- [ ] CLI interval overrides config file interval
- [ ] Validate CPU affinity ranges (0 <= cpu < num_cpus)
- [ ] Validate nice values (-20 <= nice <= 19)
- [ ] Test glob pattern matching (exact, wildcard, recursive)
- [ ] Test `should_monitor()` logic with various patterns
- [ ] Test process-specific monitor override
- [ ] Test `monitor_patterns` filtering

**TC1.2: Metric Collection Accuracy**
- [ ] CPU percentage matches `top` output (±5%, sysinfo uses different calculation)
- [ ] RSS memory matches `ps` output (exact)
- [ ] I/O bytes are monotonically increasing
- [ ] Thread count is accurate
- [ ] FD count is accurate (Linux)

**TC1.3: CSV Format Validation**
- [ ] CSV header is correct
- [ ] All fields are present
- [ ] Timestamps are ISO 8601 format
- [ ] Numbers are properly formatted
- [ ] File is valid CSV (parseable by pandas)

**TC1.4: Process Control**
- [ ] CPU affinity is applied correctly (verify via `/proc/<pid>/status`)
- [ ] Nice value is applied correctly (verify via `/proc/<pid>/stat`)
- [ ] Permission errors are handled gracefully
- [ ] Pattern matching works for different node names

**TC1.5: Monitoring Thread Stability**
- [ ] Thread runs without crashes for 1 hour
- [ ] Memory usage is stable (no leaks)
- [ ] Handles process exits gracefully
- [ ] Sampling interval is accurate (±10ms)
- [ ] Shutdown is clean (no hanging threads)

**TC1.6: Integration Test**
- [ ] Run simple ROS node with monitoring enabled
- [ ] Verify CSV file is created
- [ ] Verify metrics are collected at correct interval
- [ ] Verify process config is applied
- [ ] No performance regression (<5% overhead)

**TC1.7: Autoware System Test**
- [ ] Run Autoware with monitoring enabled
- [ ] All 52 composable nodes are monitored
- [ ] CSV files created for all nodes
- [ ] Apply process config to subset of nodes
- [ ] Verify no crashes or errors
- [ ] Total overhead < 3% CPU

---

### Phase 2: Advanced Metrics (Target: 2-3 weeks)

#### Work Items

**2.1 GPU Metrics**
- [ ] NVIDIA GPU support via `nvidia-smi` or NVML
- [ ] AMD GPU support via `rocm-smi`
- [ ] Per-process GPU memory usage
- [ ] GPU utilization percentage
- [ ] Add GPU metrics to CSV

**2.2 Network I/O**
- [ ] Per-process network stats from `/proc/<pid>/net/dev`
- [ ] TCP/UDP connection counts
- [ ] Bytes sent/received
- [ ] Add network metrics to CSV

**2.3 Aggregated Metrics**
- [ ] Container-level aggregation (sum of all nodes in container)
- [ ] System-level totals
- [ ] Summary CSV files
- [ ] Min/max/avg statistics

**2.4 Advanced Monitoring**
- [ ] Page fault counters
- [ ] Context switch counts
- [ ] Signal handling stats
- [ ] Child process tracking

#### Test Cases

**TC2.1: GPU Monitoring**
- [ ] Detect GPU correctly (NVIDIA/AMD/none)
- [ ] GPU memory matches `nvidia-smi` output
- [ ] GPU utilization is accurate
- [ ] Handles multiple GPUs
- [ ] Graceful fallback if no GPU

**TC2.2: Network Monitoring**
- [ ] Network bytes are monotonically increasing
- [ ] Connection counts are accurate
- [ ] Per-interface statistics work
- [ ] Handles network namespace isolation

**TC2.3: Aggregation Accuracy**
- [ ] Container totals equal sum of child processes
- [ ] System totals are consistent
- [ ] Summary statistics are correct (min/max/avg)

---

### Phase 3: Visualization & Analysis (Target: 3-4 weeks)

#### Work Items

**3.1 CLI Analysis Tool**
- [ ] `play_launch analyze` subcommand
- [ ] Generate summary report from CSV files
- [ ] Identify top CPU/memory consumers
- [ ] Detect anomalies (spikes, leaks)
- [ ] Export to various formats (JSON, HTML)

**3.2 Plotting Utilities**
- [ ] `play_launch plot` subcommand using plotly
- [ ] Time-series plots for CPU/memory/I/O
- [ ] Flamegraph generation
- [ ] Heatmap for CPU affinity visualization
- [ ] Comparison plots between runs

**3.3 Web Dashboard (Optional)**
- [ ] Real-time metrics streaming via WebSocket
- [ ] Interactive plots with drill-down
- [ ] Live CPU/memory gauges
- [ ] Process tree visualization
- [ ] Export/share functionality

**3.4 Documentation**
- [ ] User guide for resource monitoring
- [ ] Best practices for process config
- [ ] Visualization examples
- [ ] Performance tuning guide

#### Test Cases

**TC3.1: Analysis Accuracy**
- [ ] Summary report is accurate
- [ ] Top consumers are correctly identified
- [ ] Anomaly detection works (leak detection)

**TC3.2: Plotting**
- [ ] Plots render correctly
- [ ] Legends are clear
- [ ] Time axis is correct
- [ ] Multiple runs can be overlaid

**TC3.3: Web Dashboard**
- [ ] Dashboard loads without errors
- [ ] Real-time updates work
- [ ] Interactive controls work
- [ ] Export functionality works

## Security Considerations

- **Permissions**: Setting nice values < 0 requires CAP_SYS_NICE capability
- **CPU Affinity**: Requires CAP_SYS_NICE for other processes
- **File Access**: `/proc/<pid>/io` requires same user or root
- **Validation**: Validate configuration file to prevent privilege escalation

## Progress Tracking

Use checkboxes in the work items above to track progress. Update this document as work is completed.

### Completion Summary

- Phase 1: 0/42 tasks completed (0%)
- Phase 2: 0/19 tasks completed (0%)
- Phase 3: 0/19 tasks completed (0%)

**Total Progress: 0/80 tasks (0%)**

### Current Status

**Active Phase:** Not started
**Current Work Item:** None
**Blockers:** None
**Last Updated:** 2025-10-15

## Migration Path

1. Add monitoring as **opt-in** feature (`--enable-monitoring`)
2. Keep monitoring code isolated in separate modules
3. Gracefully handle monitoring failures (warn but continue execution)
4. Document performance impact in README

## Configuration Design Summary

### Default Behavior
- **Monitoring disabled by default** - Zero overhead unless explicitly enabled
- **All nodes monitored when enabled** - Simple default behavior
- **1 second sampling interval** - Balance between detail and overhead

### Three Ways to Enable Monitoring

1. **CLI flag only** (simplest)
   ```bash
   play_launch --enable-monitoring
   ```
   Monitors all nodes with default 1s interval.

2. **Config file only**
   ```bash
   play_launch --config config.yaml
   # or short form:
   play_launch -c config.yaml
   ```
   Config file controls everything (enabled, interval, which nodes, process control).

3. **Config file + CLI override**
   ```bash
   play_launch --config config.yaml --enable-monitoring
   ```
   Config file provides process control, CLI enables monitoring even if config says disabled.

### Configuration Flexibility

- **Monitor all nodes**: Default when monitoring is enabled
- **Monitor specific patterns**: Use `monitoring.monitor_patterns`
- **Per-node control**: Individual `monitor: true/false` in process configs
- **Process control without monitoring**: Set `monitoring.enabled: false` but configure nice/affinity
- **Monitoring without process control**: Just use `--enable-monitoring` flag

## Open Questions

1. Should we monitor composable nodes differently than regular nodes?
2. Should we aggregate metrics for nodes in the same container?
3. How to handle very short-lived processes?
4. Should we support cgroup-level metrics?
5. Should monitoring continue after process exit to capture final metrics?
6. Do we need separate user/system CPU time? (sysinfo doesn't provide this)
7. Do we need I/O operation counts? (sysinfo only provides bytes, not ops)
8. Should we create metrics directory even when no nodes are monitored?

## References

- **sysinfo crate**: https://docs.rs/sysinfo/latest/sysinfo/
- sysinfo repository: https://github.com/GuillaumeGomez/sysinfo
- `/proc` filesystem documentation: https://man7.org/linux/man-pages/man5/proc.5.html
- `sched_setaffinity`: https://man7.org/linux/man-pages/man2/sched_setaffinity.2.html
- `setpriority`: https://man7.org/linux/man-pages/man2/setpriority.2.html
