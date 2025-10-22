mod component_loader;
mod config;
mod container_readiness;
mod context;
mod execution;
mod launch_dump;
mod node_cmdline;
mod options;
mod resource_monitor;

use crate::{
    config::load_runtime_config,
    container_readiness::ServiceDiscoveryHandle,
    context::{
        prepare_composable_node_contexts, prepare_node_contexts, ComposableNodeContextSet,
        NodeContextClasses,
    },
    execution::{
        spawn_nodes, spawn_or_load_composable_nodes, ComposableNodeExecutionConfig,
        ComposableNodeTasks, SpawnComposableNodeConfig,
    },
    launch_dump::{load_and_transform_node_records, load_launch_dump, NodeContainerRecord},
    options::Options,
    resource_monitor::{spawn_monitor_thread, MonitorConfig},
};
use clap::Parser;
use eyre::Context;
use futures::{future::JoinAll, FutureExt};
use itertools::chain;
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs,
    io::{self, prelude::*},
    path::{Path, PathBuf},
    process,
    sync::{Arc, Mutex, OnceLock},
    time::Duration,
};
use tokio::runtime::Runtime;
use tracing::{debug, error, info, warn};

/// Global handle for ROS service discovery (optional, only initialized if service checking enabled)
static SERVICE_DISCOVERY_HANDLE: OnceLock<ServiceDiscoveryHandle> = OnceLock::new();

/// Kill all descendant processes of the current process recursively
#[cfg(unix)]
fn kill_all_descendants() {
    let my_pid = process::id();
    debug!("Killing all descendant processes of PID {}", my_pid);

    // Find all descendant PIDs recursively (including grandchildren)
    let descendants = find_all_descendants(my_pid);

    if !descendants.is_empty() {
        debug!(
            "Found {} descendant processes to terminate: {:?}",
            descendants.len(),
            descendants
        );

        // Kill them in reverse order (children before parents) with SIGTERM
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGTERM to PID {}", pid);
            let _ = std::process::Command::new("kill")
                .args(["-TERM", &pid.to_string()])
                .output();
        }

        // Give processes time to terminate gracefully
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Force kill any remaining processes with SIGKILL
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGKILL to PID {}", pid);
            let _ = std::process::Command::new("kill")
                .args(["-KILL", &pid.to_string()])
                .output();
        }
    } else {
        debug!("No descendant processes found to terminate");
    }
}

/// Recursively find all descendant PIDs of a given parent PID
#[cfg(unix)]
fn find_all_descendants(parent_pid: u32) -> Vec<u32> {
    let mut result = Vec::new();

    // Use pgrep to find direct children of this parent
    let output = std::process::Command::new("pgrep")
        .args(["-P", &parent_pid.to_string()])
        .output();

    if let Ok(output) = output {
        if output.status.success() {
            let stdout = String::from_utf8_lossy(&output.stdout);
            for line in stdout.lines() {
                if let Ok(child_pid) = line.trim().parse::<u32>() {
                    // Add this child
                    result.push(child_pid);
                    // Recursively find this child's descendants (grandchildren, etc.)
                    result.extend(find_all_descendants(child_pid));
                }
            }
        }
    }

    result
}

#[cfg(not(unix))]
fn kill_all_descendants() {
    // No-op on non-Unix systems
}

fn main() -> eyre::Result<()> {
    // install global collector configured based on RUST_LOG env var.
    tracing_subscriber::fmt::init();

    // Debug: Check AMENT_PREFIX_PATH at startup
    if let Ok(ament_path) = std::env::var("AMENT_PREFIX_PATH") {
        debug!(
            "AMENT_PREFIX_PATH first 200 chars: {}",
            ament_path.chars().take(200).collect::<String>()
        );
    } else {
        warn!("AMENT_PREFIX_PATH NOT SET!");
    }

    let opts = Options::parse();

    if opts.print_shell {
        generate_shell(&opts)?;
    } else {
        // Start ROS service discovery thread if service checking is enabled
        if opts.wait_for_service_ready {
            info!("Starting ROS service discovery thread for container readiness checking...");
            if opts.service_ready_timeout_secs == 0 {
                info!("Container service readiness will wait indefinitely");
            } else {
                info!(
                    "Container service readiness timeout: {}s",
                    opts.service_ready_timeout_secs
                );
            }

            match container_readiness::start_service_discovery_thread() {
                Ok(handle) => {
                    SERVICE_DISCOVERY_HANDLE
                        .set(handle)
                        .expect("SERVICE_DISCOVERY_HANDLE already set");
                    info!("ROS service discovery thread started successfully");
                }
                Err(e) => {
                    error!("Failed to start ROS service discovery thread: {}", e);
                    error!("Falling back to process-based container checking");
                }
            }
        } else {
            info!("Service readiness checking disabled (use --wait-for-service-ready to enable)");
        }

        // Build the async runtime.
        let runtime = Runtime::new()?;

        // Run the whole playing task in the runtime.
        runtime.block_on(play(&opts))?;
    }

    Ok(())
}

/// Generate shell script from the launch record.
fn generate_shell(opts: &options::Options) -> eyre::Result<()> {
    let log_dir = create_log_dir(&opts.log_dir)?;
    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;

    let launch_dump = load_launch_dump(&opts.input_file)
        .wrap_err_with(|| format!("unable to read launch file {}", opts.input_file.display()))?;

    let process_records = load_and_transform_node_records(&launch_dump, &params_files_dir)?;
    let process_shells = process_records
        .par_iter()
        .map(|(_record, cmdline)| cmdline.to_shell(false));

    let load_node_shells = launch_dump
        .load_node
        .par_iter()
        .map(|request| request.to_shell(opts.standalone_composable_nodes));

    let mut shells: Vec<_> = process_shells.chain(load_node_shells).collect();
    shells.par_sort_unstable();

    let mut stdout = io::stdout();
    writeln!(stdout, "#!/bin/sh")?;
    for shell in shells {
        stdout.write_all(&shell)?;
        stdout.write_all(b"\n")?;
    }

    Ok(())
}

/// Guard that ensures child processes are cleaned up on drop
struct CleanupGuard;

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        debug!("CleanupGuard: Ensuring all child processes are terminated");
        kill_all_descendants();
    }
}

/// Play the launch according to the launch record.
async fn play(opts: &options::Options) -> eyre::Result<()> {
    // Install cleanup guard to ensure children are killed even if we're interrupted
    let _cleanup_guard = CleanupGuard;

    // Load runtime configuration
    let runtime_config = load_runtime_config(
        opts.config.as_deref(),
        opts.enable_monitoring,
        opts.monitor_interval_ms,
    )?;

    let launch_dump = load_launch_dump(&opts.input_file)?;

    // Prepare directories
    let log_dir = create_log_dir(&opts.log_dir)?;

    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", node_log_dir.display()))?;

    let load_node_log_dir = log_dir.join("load_node");
    fs::create_dir(&load_node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", load_node_log_dir.display()))?;

    // Initialize NVML for GPU monitoring
    let nvml = match nvml_wrapper::Nvml::init() {
        Ok(nvml) => {
            let device_count = nvml.device_count().unwrap_or(0);
            info!(
                "NVML initialized successfully with {} GPU device(s)",
                device_count
            );
            Some(nvml)
        }
        Err(e) => {
            error!("Failed to initialize NVML: {}", e);
            error!("GPU monitoring will be unavailable. Ensure NVIDIA drivers are installed.");
            None
        }
    };

    // Initialize monitoring if enabled
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, String>::new()));
    let _monitor_thread = if runtime_config.monitoring.enabled {
        let monitor_config = MonitorConfig {
            enabled: true,
            sample_interval_ms: runtime_config.monitoring.sample_interval_ms,
            log_dir: log_dir.clone(),
        };
        match spawn_monitor_thread(monitor_config, process_registry.clone(), nvml) {
            Ok(handle) => {
                info!(
                    "Resource monitoring enabled (interval: {}ms)",
                    runtime_config.monitoring.sample_interval_ms
                );
                Some(handle)
            }
            Err(e) => {
                error!("Failed to start monitoring thread: {}", e);
                None
            }
        }
    } else {
        debug!("Resource monitoring disabled");
        None
    };

    // Build a table of composable node containers
    let container_names: HashSet<String> = launch_dump
        .container
        .par_iter()
        .map(|record| {
            let NodeContainerRecord { namespace, name } = record;
            // Handle namespace ending with '/' to avoid double slashes
            if namespace.ends_with('/') {
                format!("{namespace}{name}")
            } else {
                format!("{namespace}/{name}")
            }
        })
        .collect();

    // Prepare node execution contexts
    let NodeContextClasses {
        container_contexts,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)?;

    // Prepare LoadNode request execution contexts
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;

    // Report the number of entities
    info!("nodes: {}", pure_node_contexts.len());

    // Spawn non-container nodes
    let non_container_node_tasks = spawn_nodes(pure_node_contexts, Some(process_registry.clone()))
        .into_iter()
        .map(|future| future.boxed());

    // Initialize component loader for service-based loading
    info!("Initializing component loader for service-based node loading");
    let component_loader = match crate::component_loader::start_component_loader_thread() {
        Ok(loader) => Some(loader),
        Err(e) => {
            error!("Failed to initialize component loader: {}", e);
            None
        }
    };

    // Create composable node execution configuration
    let composable_node_config = ComposableNodeExecutionConfig {
        standalone_composable_nodes: opts.standalone_composable_nodes,
        load_orphan_composable_nodes: opts.load_orphan_composable_nodes,
        spawn_config: SpawnComposableNodeConfig {
            max_concurrent_spawn: opts.max_concurrent_load_node_spawn,
            max_attempts: opts.load_node_attempts,
            wait_timeout: Duration::from_millis(opts.load_node_timeout_millis),
        },
        load_node_delay: Duration::from_millis(opts.delay_load_node_millis),
        service_wait_config: if opts.wait_for_service_ready {
            Some(crate::container_readiness::ContainerWaitConfig::new(
                opts.service_ready_timeout_secs,
                opts.service_poll_interval_ms,
            ))
        } else {
            None
        },
        component_loader,
        process_registry: Some(process_registry.clone()),
        process_configs: runtime_config.monitoring.process_configs,
    };

    // Create the task set to load composable nodes according to user
    // options.
    let composable_node_tasks = spawn_or_load_composable_nodes(
        container_contexts,
        load_node_contexts,
        &container_names,
        composable_node_config,
    );

    // Unpack the task set to a Vec of tasks.
    let wait_composable_node_tasks: Vec<_> = match composable_node_tasks {
        ComposableNodeTasks::Standalone {
            wait_composable_node_tasks,
        } => wait_composable_node_tasks,
        ComposableNodeTasks::Container {
            wait_container_tasks,
            load_nice_composable_nodes_task,
            load_orphan_composable_nodes_task,
        } => {
            let load_task = async move {
                info!("Loading composable nodes...");

                let join: JoinAll<_> = chain!(
                    [load_nice_composable_nodes_task.boxed()],
                    load_orphan_composable_nodes_task.map(|task| task.boxed())
                )
                .collect();
                join.await;

                info!("Done loading all composable nodes");
                eyre::Ok(())
            };

            chain!(wait_container_tasks, [load_task.boxed()]).collect()
        }
    };

    // Collect all waiting tasks built so far.
    let mut wait_futures: Vec<_> =
        chain!(non_container_node_tasks, wait_composable_node_tasks).collect();

    // Poll on all waiting tasks and consume finished tasks
    // one-by-one, while also listening for termination signals.

    #[cfg(unix)]
    let result = {
        let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("Failed to register SIGTERM handler");

        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Received SIGINT (Ctrl-C), shutting down gracefully...");
                kill_all_descendants();
                drop(wait_futures);
                info!("All child processes terminated");
                Ok(())
            }
            _ = sigterm.recv() => {
                info!("Received SIGTERM, shutting down gracefully...");
                kill_all_descendants();
                drop(wait_futures);
                info!("All child processes terminated");
                Ok(())
            }
            _ = async {
                while !wait_futures.is_empty() {
                    let (result, ix, _) = futures::future::select_all(&mut wait_futures).await;
                    if let Err(err) = result {
                        error!("{err}");
                    }
                    let future_to_discard = wait_futures.remove(ix);
                    drop(future_to_discard);
                }
            } => {
                info!("All tasks completed normally");
                Ok(())
            }
        }
    };

    #[cfg(not(unix))]
    let result = tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            info!("Received SIGINT (Ctrl-C), shutting down gracefully...");
            kill_all_descendants();
            drop(wait_futures);
            info!("All child processes terminated");
            Ok(())
        }
        _ = async {
            while !wait_futures.is_empty() {
                let (result, ix, _) = futures::future::select_all(&mut wait_futures).await;
                if let Err(err) = result {
                    error!("{err}");
                }
                let future_to_discard = wait_futures.remove(ix);
                drop(future_to_discard);
            }
        } => {
            info!("All tasks completed normally");
            Ok(())
        }
    };

    result
}

/// Format current timestamp as YYYY-MM-DD_HH-MM-SS
fn format_timestamp() -> String {
    chrono::Local::now().format("%Y-%m-%d_%H-%M-%S").to_string()
}

/// Create a directory to store logging data.
///
/// Creates a timestamped subdirectory under the base log directory.
/// Format: base_dir/YYYY-MM-DD_HH-MM-SS/ or base_dir/YYYY-MM-DD_HH-MM-SS-N/ if conflicts occur
pub fn create_log_dir(log_dir: &Path) -> eyre::Result<PathBuf> {
    // Create base directory if it doesn't exist
    if !log_dir.exists() {
        fs::create_dir_all(log_dir)
            .wrap_err_with(|| format!("unable to create base directory {}", log_dir.display()))?;
    }

    // Create timestamped subdirectory
    let timestamp = format_timestamp();
    let mut timestamped_dir = log_dir.join(&timestamp);

    // If directory already exists, add -1, -2, ... suffix
    if timestamped_dir.exists() {
        for n in 1..=1000 {
            timestamped_dir = log_dir.join(format!("{}-{}", timestamp, n));
            if !timestamped_dir.exists() {
                break;
            }
        }

        // Check if we exhausted all attempts
        if timestamped_dir.exists() {
            eyre::bail!(
                "unable to find available timestamped directory after 1000 attempts for timestamp {}",
                timestamp
            );
        }
    }

    fs::create_dir(&timestamped_dir)
        .wrap_err_with(|| format!("unable to create directory {}", timestamped_dir.display()))?;

    Ok(timestamped_dir)
}
