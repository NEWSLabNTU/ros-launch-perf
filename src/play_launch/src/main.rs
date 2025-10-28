mod ament_index;
mod component_loader;
mod config;
mod container_readiness;
mod context;
mod dump_launcher;
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
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGTERM);
        }

        // Give processes time to terminate gracefully
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Force kill any remaining processes with SIGKILL
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGKILL to PID {}", pid);
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGKILL);
        }
    } else {
        debug!("No descendant processes found to terminate");
    }
}

/// Recursively find all descendant PIDs of a given parent PID
#[cfg(unix)]
fn find_all_descendants(parent_pid: u32) -> Vec<u32> {
    use sysinfo::System;

    let mut result = Vec::new();
    let mut sys = System::new();
    sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);

    // Find all processes that have parent_pid as their parent
    for (pid, process) in sys.processes() {
        if let Some(parent) = process.parent() {
            if parent.as_u32() == parent_pid {
                let child_pid = pid.as_u32();
                // Add this child
                result.push(child_pid);
                // Recursively find this child's descendants (grandchildren, etc.)
                result.extend(find_all_descendants(child_pid));
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

    // Route to appropriate handler based on subcommand
    match &opts.command {
        options::Command::Launch(args) => {
            handle_launch(args)?;
        }
        options::Command::Run(args) => {
            handle_run(args)?;
        }
        options::Command::Dump(args) => {
            handle_dump(args)?;
        }
        options::Command::Replay(args) => {
            handle_replay(args)?;
        }
    }

    Ok(())
}

/// Handle the 'launch' subcommand (dump + replay)
fn handle_launch(args: &options::LaunchArgs) -> eyre::Result<()> {
    use crate::dump_launcher::DumpLauncher;
    use tokio::runtime::Runtime;

    info!("Step 1/2: Recording launch execution...");

    // Create tokio runtime for async operations
    let runtime = Runtime::new()?;

    // Run dump phase
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        // Use default record.json in current directory
        let record_path = PathBuf::from("record.json");

        launcher
            .dump_launch(
                &args.package_or_path,
                args.launch_file.as_deref(),
                &args.launch_arguments,
                &record_path,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    info!("Step 2/2: Replaying launch execution...");

    // Create replay args and call handle_replay
    let replay_args = options::ReplayArgs {
        input_file: PathBuf::from("record.json"),
        common: options::CommonOptions::default(),
    };

    handle_replay(&replay_args)?;

    Ok(())
}

/// Handle the 'run' subcommand (direct node execution)
fn handle_run(args: &options::RunArgs) -> eyre::Result<()> {
    use crate::launch_dump::{LaunchDump, NodeRecord};
    use tokio::runtime::Runtime;

    info!("Running single node: {} {}", args.package, args.executable);

    // Build command line for the node
    let mut cmd = vec![
        "ros2".to_string(),
        "run".to_string(),
        args.package.clone(),
        args.executable.clone(),
    ];
    cmd.extend(args.args.clone());

    // Create a minimal LaunchDump with a single node
    let node_record = NodeRecord {
        executable: args.executable.clone(),
        package: Some(args.package.clone()),
        name: Some(args.executable.clone()),
        namespace: Some("/".to_string()),
        exec_name: Some(args.executable.clone()),
        params: vec![],
        params_files: vec![],
        remaps: vec![],
        ros_args: None,
        args: Some(args.args.clone()),
        cmd,
        env: None,
    };

    let launch_dump = LaunchDump {
        node: vec![node_record],
        load_node: vec![],
        container: vec![],
        lifecycle_node: vec![],
        file_data: HashMap::new(),
    };

    // Create minimal common options for direct execution
    let common = options::CommonOptions::default();

    // Build the async runtime and run directly
    let runtime = Runtime::new()?;
    runtime.block_on(run_direct(&launch_dump, &common))?;

    Ok(())
}

/// Run a launch dump directly without file I/O
async fn run_direct(
    launch_dump: &launch_dump::LaunchDump,
    common: &options::CommonOptions,
) -> eyre::Result<()> {
    info!("=== Starting direct node execution ===");

    // Install cleanup guard
    let _cleanup_guard = CleanupGuard;
    info!("CleanupGuard installed");

    // Load runtime configuration
    info!("Loading runtime configuration...");
    let runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.enable_monitoring,
        common.monitor_interval_ms,
    )?;
    info!("Runtime configuration loaded successfully");

    // Create temporary log directory
    info!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    info!("Log directory created: {}", log_dir.display());

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)?;
    info!("Created node log directory");

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
            None
        }
    };

    // Initialize monitoring
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
    let _monitor_thread = if runtime_config.monitoring.enabled {
        let monitor_config = MonitorConfig {
            enabled: true,
            sample_interval_ms: runtime_config.monitoring.sample_interval_ms,
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

    // Prepare node execution contexts
    let container_names = HashSet::new();
    let NodeContextClasses {
        container_contexts: _,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(launch_dump, &node_log_dir, &container_names)?;

    info!("Spawning node...");
    let node_tasks = spawn_nodes(pure_node_contexts, Some(process_registry.clone()))
        .into_iter()
        .map(|future| future.boxed());

    let mut wait_futures: Vec<_> = node_tasks.collect();
    info!("Collected {} futures to wait on", wait_futures.len());

    // Wait for node to complete or receive signal
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

/// Handle the 'dump' subcommand (dump only, no replay)
fn handle_dump(args: &options::DumpArgs) -> eyre::Result<()> {
    use crate::dump_launcher::DumpLauncher;
    use tokio::runtime::Runtime;

    info!("Recording launch execution (dump only, no replay)...");

    // Create tokio runtime for async operations
    let runtime = Runtime::new()?;

    // Run dump phase
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        match &args.subcommand {
            options::DumpSubcommand::Launch(launch_args) => {
                launcher
                    .dump_launch(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    )
                    .await?;
            }
            options::DumpSubcommand::Run(run_args) => {
                launcher
                    .dump_run(
                        &run_args.package,
                        &run_args.executable,
                        &run_args.args,
                        &args.output,
                    )
                    .await?;
            }
        }

        Ok::<(), eyre::Report>(())
    })?;

    info!("Dump completed successfully: {}", args.output.display());
    info!(
        "To replay: play_launch replay --input-file {}",
        args.output.display()
    );

    Ok(())
}

/// Handle the 'replay' subcommand
fn handle_replay(args: &options::ReplayArgs) -> eyre::Result<()> {
    let input_file = &args.input_file;

    if args.common.print_shell {
        generate_shell(input_file, &args.common)?;
    } else {
        // Start ROS service discovery thread if service checking is enabled
        if args.common.wait_for_service_ready {
            info!("Starting ROS service discovery thread for container readiness checking...");
            if args.common.service_ready_timeout_secs == 0 {
                info!("Container service readiness will wait indefinitely");
            } else {
                info!(
                    "Container service readiness timeout: {}s",
                    args.common.service_ready_timeout_secs
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
        runtime.block_on(play(input_file, &args.common))?;
    }

    Ok(())
}

/// Generate shell script from the launch record.
fn generate_shell(input_file: &Path, common: &options::CommonOptions) -> eyre::Result<()> {
    let log_dir = create_log_dir(&common.log_dir)?;
    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;

    let launch_dump = load_launch_dump(input_file)
        .wrap_err_with(|| format!("unable to read launch file {}", input_file.display()))?;

    let process_records = load_and_transform_node_records(&launch_dump, &params_files_dir)?;
    let process_shells = process_records
        .par_iter()
        .map(|(_record, cmdline)| cmdline.to_shell(false));

    let load_node_shells = launch_dump
        .load_node
        .par_iter()
        .map(|request| request.to_shell(common.standalone_composable_nodes));

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
async fn play(input_file: &Path, common: &options::CommonOptions) -> eyre::Result<()> {
    info!("=== Starting play() function ===");

    // Install cleanup guard to ensure children are killed even if we're interrupted
    let _cleanup_guard = CleanupGuard;
    info!("CleanupGuard installed");

    // Load runtime configuration
    info!("Loading runtime configuration...");
    let runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.enable_monitoring,
        common.monitor_interval_ms,
    )?;
    info!("Runtime configuration loaded successfully");

    info!("Loading launch dump from: {}", input_file.display());
    let launch_dump = load_launch_dump(input_file)?;
    info!("Launch dump loaded successfully");

    // Prepare directories
    info!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    info!("Log directory created: {}", log_dir.display());

    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;
    info!("Created params_files directory");

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", node_log_dir.display()))?;
    info!("Created node log directory");

    let load_node_log_dir = log_dir.join("load_node");
    fs::create_dir(&load_node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", load_node_log_dir.display()))?;
    info!("Created load_node log directory");

    // Initialize NVML for GPU monitoring
    info!("Initializing NVML...");
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
    info!("Initializing monitoring and process registry...");
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
    let _monitor_thread = if runtime_config.monitoring.enabled {
        let monitor_config = MonitorConfig {
            enabled: true,
            sample_interval_ms: runtime_config.monitoring.sample_interval_ms,
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
    info!("Monitoring initialization complete");

    // Build a table of composable node containers
    info!("Building container names table...");
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
    info!(
        "Container names table built with {} containers",
        container_names.len()
    );

    // Prepare node execution contexts
    info!("Preparing node execution contexts...");
    let NodeContextClasses {
        container_contexts,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)?;
    info!(
        "Node contexts prepared: {} containers, {} pure nodes",
        container_contexts.len(),
        pure_node_contexts.len()
    );

    // Prepare LoadNode request execution contexts
    info!("Preparing composable node contexts...");
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;
    info!(
        "Composable node contexts prepared: {} load_node contexts",
        load_node_contexts.len()
    );

    // Report the number of entities
    info!("nodes: {}", pure_node_contexts.len());

    // Spawn non-container nodes
    info!("Spawning non-container nodes...");
    let non_container_node_tasks = spawn_nodes(pure_node_contexts, Some(process_registry.clone()))
        .into_iter()
        .map(|future| future.boxed());

    // Initialize component loader for service-based loading
    info!("Initializing component loader for service-based node loading");
    let component_loader = match crate::component_loader::start_component_loader_thread() {
        Ok(loader) => {
            info!("Component loader initialized successfully");
            Some(loader)
        }
        Err(e) => {
            error!("Failed to initialize component loader: {}", e);
            error!("Continuing without component loader");
            None
        }
    };

    info!("Proceeding with execution...");

    // Create composable node execution configuration
    let composable_node_config = ComposableNodeExecutionConfig {
        standalone_composable_nodes: common.standalone_composable_nodes,
        load_orphan_composable_nodes: common.load_orphan_composable_nodes,
        spawn_config: SpawnComposableNodeConfig {
            max_concurrent_spawn: common.max_concurrent_load_node_spawn,
            max_attempts: common.load_node_attempts,
            wait_timeout: Duration::from_millis(common.load_node_timeout_millis),
        },
        load_node_delay: Duration::from_millis(common.delay_load_node_millis),
        service_wait_config: if common.wait_for_service_ready {
            Some(crate::container_readiness::ContainerWaitConfig::new(
                common.service_ready_timeout_secs,
                common.service_poll_interval_ms,
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

    info!("Collected {} futures to wait on", wait_futures.len());
    if wait_futures.is_empty() {
        warn!("No futures to wait on - this will cause immediate exit!");
    }

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
