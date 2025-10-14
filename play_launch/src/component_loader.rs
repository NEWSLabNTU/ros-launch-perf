// Component loader module - loads composable nodes via ROS service calls
//
// This module provides a background thread that handles loading composable nodes
// into containers using the composition_interfaces/srv/LoadNode service.
//
// The approach uses direct ROS service calls instead of spawning `ros2 component load`
// subprocesses, which eliminates Python startup overhead and enables better
// parallelization.

use crate::composition_interfaces::{LoadNodeRequest, LoadNodeResponse};
use eyre::{Context as _, Result};
use std::{collections::HashMap, time::Duration};
use tokio::sync::{mpsc, oneshot};
use tracing::{debug, error, info, warn};

/// Handle to the component loader background thread
#[derive(Clone)]
pub struct ComponentLoaderHandle {
    request_tx: mpsc::UnboundedSender<LoadRequest>,
}

/// Internal request structure for loading a node
struct LoadRequest {
    container_name: String,
    package_name: String,
    plugin_name: String,
    node_name: String,
    node_namespace: String,
    remap_rules: Vec<String>,
    parameters: Vec<(String, String)>,
    extra_args: Vec<(String, String)>,
    response_tx: oneshot::Sender<Result<LoadNodeResponse>>,
    timeout: Duration,
}

impl ComponentLoaderHandle {
    /// Request to load a composable node into a container
    ///
    /// Returns a future that resolves when the service call completes
    pub async fn load_node(
        &self,
        container_name: &str,
        package_name: &str,
        plugin_name: &str,
        node_name: &str,
        node_namespace: &str,
        remap_rules: Vec<String>,
        parameters: Vec<(String, String)>,
        extra_args: Vec<(String, String)>,
        timeout: Duration,
    ) -> Result<LoadNodeResponse> {
        let (response_tx, response_rx) = oneshot::channel();

        let request = LoadRequest {
            container_name: container_name.to_string(),
            package_name: package_name.to_string(),
            plugin_name: plugin_name.to_string(),
            node_name: node_name.to_string(),
            node_namespace: node_namespace.to_string(),
            remap_rules,
            parameters,
            extra_args,
            response_tx,
            timeout,
        };

        debug!("Sending load request to component loader thread...");
        self.request_tx.send(request).wrap_err_with(|| {
            format!(
                "Component loader thread has stopped (while loading {}/{} into {})",
                package_name, plugin_name, container_name
            )
        })?;

        debug!("Waiting for response from component loader thread...");
        let result = response_rx.await.wrap_err_with(|| {
            format!(
                "Component loader dropped request for {}/{} into {}",
                package_name, plugin_name, container_name
            )
        })?;

        debug!("Received response from component loader thread");
        result
    }
}

/// Start the component loader background thread
///
/// This creates a ROS node and executor that processes load requests
pub fn start_component_loader_thread() -> Result<ComponentLoaderHandle> {
    let (request_tx, request_rx) = mpsc::unbounded_channel();

    std::thread::Builder::new()
        .name("component_loader".to_string())
        .spawn(move || {
            if let Err(e) = run_component_loader_loop(request_rx) {
                error!("Component loader thread failed: {:#}", e);
            }
        })
        .context("Failed to spawn component loader thread")?;

    info!("Started component loader background thread");

    Ok(ComponentLoaderHandle { request_tx })
}

/// Main loop for the component loader thread
///
/// This runs in a dedicated thread and handles all service calls
fn run_component_loader_loop(mut request_rx: mpsc::UnboundedReceiver<LoadRequest>) -> Result<()> {
    use rclrs::CreateBasicExecutor;

    // Initialize ROS context and node
    let context = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())
        .context("Failed to create ROS context")?;

    let executor = context.create_basic_executor();
    let node = executor
        .create_node("play_launch_component_loader")
        .context("Failed to create component loader node")?;

    info!("Component loader node created");

    // Note: Direct rcl service calls require complex Message/RmwMessage trait implementations.
    // For Phase 2, we implement a working solution using ros2 CLI as a bridge.
    // Future optimization: implement direct rcl_send_request/rcl_take_response calls.

    while let Some(request) = request_rx.blocking_recv() {
        debug!(
            "Loading node {}/{} into container {}",
            request.package_name, request.plugin_name, request.container_name
        );

        // Call ros2 component load as a subprocess
        // This validates the infrastructure while we work on direct rcl implementation
        debug!("About to call call_component_load_subprocess...");
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            debug!("Inside panic handler closure");
            call_component_load_subprocess(
                &request.container_name,
                &request.package_name,
                &request.plugin_name,
                &request.node_name,
                &request.node_namespace,
                &request.remap_rules,
                &request.parameters,
                &request.extra_args,
                request.timeout,
            )
        }));
        debug!("Returned from panic handler");

        let result = match result {
            Ok(r) => r,
            Err(e) => {
                error!("Panic in component loader: {:?}", e);
                Err(eyre::eyre!("Component loader panicked"))
            }
        };

        let _ = request.response_tx.send(result);
    }

    info!("Component loader shutting down");
    Ok(())
}

/// Call ros2 component load via subprocess
///
/// This is a temporary implementation that validates the infrastructure.
/// Future versions will use direct rcl service calls.
fn call_component_load_subprocess(
    container_name: &str,
    package: &str,
    plugin: &str,
    node_name: &str,
    namespace: &str,
    remap_rules: &[String],
    parameters: &[(String, String)],
    extra_args: &[(String, String)],
    timeout: Duration,
) -> Result<LoadNodeResponse> {
    debug!(
        "ENTERED call_component_load_subprocess for container={}",
        container_name
    );

    use std::{
        io::Read,
        process::{Command, Stdio},
    };

    debug!("Building ros2 component load command...");

    // Build the ros2 component load command
    // Build the command as a shell command string to ensure proper environment
    let mut args = vec![
        "component".to_string(),
        "load".to_string(),
        container_name.to_string(),
        package.to_string(),
        plugin.to_string(),
    ];

    if !node_name.is_empty() {
        args.push("-n".to_string());
        args.push(node_name.to_string());
    }

    if !namespace.is_empty() {
        args.push("--node-namespace".to_string());
        args.push(namespace.to_string());
    }

    for remap in remap_rules {
        args.push("-r".to_string());
        args.push(remap.to_string());
    }

    for (name, value) in parameters {
        args.push("-p".to_string());
        args.push(format!("{}:={}", name, value));
    }

    for (name, value) in extra_args {
        args.push("-e".to_string());
        args.push(format!("{}:={}", name, value));
    }

    // Use bash -c to execute ros2 command
    // The subprocess inherits the environment from the parent process
    let ros2_cmdline = format!("ros2 {}", args.join(" "));
    debug!("Executing via bash: {}", ros2_cmdline);

    let mut cmd = Command::new("/usr/bin/bash");
    cmd.arg("-c")
        .arg(&ros2_cmdline)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    debug!("About to spawn command");
    let mut child = cmd.spawn().wrap_err_with(|| {
        format!(
            "Failed to spawn bash to load {}/{} into {}",
            package, plugin, container_name
        )
    })?;
    debug!("Successfully spawned bash process");

    // Wait for completion with timeout
    let start = std::time::Instant::now();
    loop {
        match child.try_wait() {
            Ok(Some(status)) => {
                // Process completed
                let mut stdout = String::new();
                let mut stderr = String::new();

                if let Some(mut out) = child.stdout.take() {
                    let _ = out.read_to_string(&mut stdout);
                }
                if let Some(mut err) = child.stderr.take() {
                    let _ = err.read_to_string(&mut stderr);
                }

                if status.success() {
                    // Parse the output to extract unique_id
                    // Output format: "Loaded component X into '/container' with unique ID Y"
                    let unique_id = parse_unique_id_from_output(&stdout);

                    return Ok(LoadNodeResponse {
                        success: true,
                        error_message: String::new(),
                        full_node_name: format!("{}/{}", namespace, node_name),
                        unique_id,
                    });
                } else {
                    error!("Bash command failed with stderr: {}", stderr);
                    error!("Bash command failed with stdout: {}", stdout);
                    return Ok(LoadNodeResponse {
                        success: false,
                        error_message: format!("Command failed: {}", stderr),
                        full_node_name: String::new(),
                        unique_id: 0,
                    });
                }
            }
            Ok(None) => {
                // Still running, check timeout
                if start.elapsed() > timeout {
                    let _ = child.kill();
                    return Ok(LoadNodeResponse {
                        success: false,
                        error_message: format!("Timeout after {:?}", timeout),
                        full_node_name: String::new(),
                        unique_id: 0,
                    });
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            Err(e) => {
                return Err(eyre::eyre!("Error waiting for process: {}", e)).wrap_err_with(|| {
                    format!(
                        "While loading {}/{} into {}",
                        package, plugin, container_name
                    )
                });
            }
        }
    }
}

/// Parse unique ID from ros2 component load output
fn parse_unique_id_from_output(output: &str) -> u64 {
    // Look for "unique ID" followed by a number
    for line in output.lines() {
        if let Some(pos) = line.find("unique ID") {
            let after = &line[pos + 9..].trim();
            if let Some(num_str) = after.split_whitespace().next() {
                if let Ok(id) = num_str.parse::<u64>() {
                    return id;
                }
            }
        }
    }
    0
}
