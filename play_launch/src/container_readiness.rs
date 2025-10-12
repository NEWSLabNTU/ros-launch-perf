//! Container readiness checking module
//!
//! **STATUS: Currently Unused**
//!
//! This module contains ROS service discovery code for checking container readiness.
//! It was implemented using rclrs native service discovery but disabled because:
//! - DDS service discovery is unreliable/extremely slow in some environments (e.g., Autoware)
//! - `ros2 service list` and `rclrs::Node::get_service_names_and_types()` can hang indefinitely
//! - Process-based checking (verifying container PIDs) is more reliable and instant
//!
//! The code is kept for potential future use if DDS discovery issues are resolved.
//!
//! **Alternative Implementation:** See `execution::wait_for_containers_running()` which uses
//! process PID checking instead of service discovery.

#![allow(dead_code)]

use std::time::Duration;
use tokio::sync::{mpsc, oneshot};
use tracing::{debug, error, info, warn};

/// Configuration for waiting for container readiness
#[derive(Clone, Debug)]
pub struct ContainerWaitConfig {
    pub max_wait: Duration,
    pub poll_interval: Duration,
}

impl ContainerWaitConfig {
    pub fn new(timeout_secs: u64, poll_interval_ms: u64) -> Self {
        Self {
            max_wait: Duration::from_secs(timeout_secs),
            poll_interval: Duration::from_millis(poll_interval_ms),
        }
    }
}

/// Request to check if a service exists
#[derive(Debug)]
struct ServiceCheckRequest {
    service_name: String,
    response_tx: oneshot::Sender<bool>,
}

/// Handle for communicating with the ROS service discovery thread
#[derive(Clone, Debug)]
pub struct ServiceDiscoveryHandle {
    request_tx: mpsc::UnboundedSender<ServiceCheckRequest>,
}

impl ServiceDiscoveryHandle {
    /// Check if a service exists
    pub async fn check_service_exists(&self, service_name: String) -> eyre::Result<bool> {
        let (response_tx, response_rx) = oneshot::channel();

        self.request_tx
            .send(ServiceCheckRequest {
                service_name,
                response_tx,
            })
            .map_err(|_| eyre::eyre!("ROS discovery thread has stopped"))?;

        response_rx
            .await
            .map_err(|_| eyre::eyre!("Failed to receive response from ROS discovery thread"))
    }
}

/// Start the ROS service discovery thread
/// Returns a handle for querying service existence
pub fn start_service_discovery_thread() -> eyre::Result<ServiceDiscoveryHandle> {
    let (request_tx, request_rx) = mpsc::unbounded_channel();

    std::thread::Builder::new()
        .name("ros_discovery".to_string())
        .spawn(move || {
            if let Err(e) = run_ros_discovery_loop(request_rx) {
                error!("ROS discovery thread error: {}", e);
            }
        })?;

    Ok(ServiceDiscoveryHandle { request_tx })
}

/// Run the ROS discovery loop in a dedicated thread
fn run_ros_discovery_loop(
    mut request_rx: mpsc::UnboundedReceiver<ServiceCheckRequest>,
) -> eyre::Result<()> {
    use rclrs::CreateBasicExecutor;

    info!("Starting ROS service discovery thread");

    // Initialize ROS context and node
    let context = rclrs::Context::new(std::env::args(), rclrs::InitOptions::default())?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("play_launch_discovery")?;

    info!("ROS service discovery node initialized");

    // Process requests
    while let Some(request) = request_rx.blocking_recv() {
        let ServiceCheckRequest {
            service_name,
            response_tx,
        } = request;

        // Query service existence
        let exists = match node.get_service_names_and_types() {
            Ok(services) => {
                let found = services.contains_key(&service_name);

                if !found {
                    // Log all available services to help debug
                    debug!("Service '{}' not found. Available services ({}):",
                           service_name, services.len());

                    // Show container-related services for debugging
                    let container_services: Vec<_> = services.keys()
                        .filter(|k| k.contains("_container") || k.contains("load_node"))
                        .collect();

                    if !container_services.is_empty() {
                        debug!("  Container-related services found:");
                        for svc in container_services.iter().take(10) {
                            debug!("    - {}", svc);
                        }
                        if container_services.len() > 10 {
                            debug!("    ... and {} more", container_services.len() - 10);
                        }
                    } else {
                        debug!("  No container-related services found yet");
                    }
                } else {
                    info!("Service '{}' found!", service_name);
                }

                found
            }
            Err(e) => {
                warn!("Failed to get service names: {}", e);
                false
            }
        };

        // Send response (ignore if receiver dropped)
        let _ = response_tx.send(exists);
    }

    info!("ROS service discovery thread shutting down");
    Ok(())
}

/// Wait for all containers to be ready by checking for their _container/load_node services
pub async fn wait_for_containers_ready(
    container_names: &[String],
    config: &ContainerWaitConfig,
    discovery_handle: &ServiceDiscoveryHandle,
) -> eyre::Result<()> {
    if container_names.is_empty() {
        return Ok(());
    }

    info!(
        "Waiting for {} container(s) to be ready...",
        container_names.len()
    );

    let mut tasks = vec![];

    for container_name in container_names {
        let name = container_name.clone();
        let cfg = config.clone();
        let handle = discovery_handle.clone();

        let task =
            tokio::spawn(async move { wait_for_single_container(&name, &cfg, &handle).await });

        tasks.push(task);
    }

    // Wait for all containers in parallel
    for (idx, task) in tasks.into_iter().enumerate() {
        match task.await {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                warn!(
                    "Error waiting for container {}: {}",
                    container_names[idx], e
                );
            }
            Err(e) => {
                warn!(
                    "Task panicked for container {}: {}",
                    container_names[idx], e
                );
            }
        }
    }

    info!("All containers ready (or timed out)");
    Ok(())
}

/// Wait for a single container to be ready by polling for its load_node service
async fn wait_for_single_container(
    container_name: &str,
    config: &ContainerWaitConfig,
    discovery_handle: &ServiceDiscoveryHandle,
) -> eyre::Result<()> {
    let service_name = format!("{}/_container/load_node", container_name);
    let deadline = tokio::time::Instant::now() + config.max_wait;

    debug!(
        "Waiting for container '{}' to expose service '{}'",
        container_name, service_name
    );

    loop {
        // Check if the service is available
        match discovery_handle
            .check_service_exists(service_name.clone())
            .await
        {
            Ok(true) => {
                info!("Container '{}' is ready", container_name);
                return Ok(());
            }
            Ok(false) => {
                // Service not yet available, continue polling
            }
            Err(e) => {
                warn!(
                    "Error checking service availability for '{}': {}",
                    container_name, e
                );
                // Continue polling despite error
            }
        }

        // Check timeout
        if tokio::time::Instant::now() >= deadline {
            warn!(
                "Timeout waiting for container '{}' after {:?}, proceeding anyway",
                container_name, config.max_wait
            );
            return Ok(());
        }

        tokio::time::sleep(config.poll_interval).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_container_wait_config() {
        let config = ContainerWaitConfig::new(10, 200);
        assert_eq!(config.max_wait, Duration::from_secs(10));
        assert_eq!(config.poll_interval, Duration::from_millis(200));
    }
}
