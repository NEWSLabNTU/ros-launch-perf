use std::time::Duration;
use tracing::{debug, info, warn};

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

/// Wait for all containers to be ready by checking for their _container/load_node services
pub async fn wait_for_containers_ready(
    container_names: &[String],
    config: &ContainerWaitConfig,
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

        let task = tokio::spawn(async move { wait_for_single_container(&name, &cfg).await });

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
) -> eyre::Result<()> {
    let service_name = format!("{}/_container/load_node", container_name);
    let deadline = tokio::time::Instant::now() + config.max_wait;

    debug!(
        "Waiting for container '{}' to expose service '{}'",
        container_name, service_name
    );

    loop {
        // Check if the service is available
        match check_service_exists(&service_name).await {
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

/// Check if a ROS2 service exists by calling `ros2 service list`
async fn check_service_exists(service_name: &str) -> eyre::Result<bool> {
    let output = tokio::process::Command::new("ros2")
        .args(["service", "list"])
        .output()
        .await?;

    if !output.status.success() {
        return Ok(false);
    }

    let services = String::from_utf8_lossy(&output.stdout);
    let exists = services.lines().any(|line| line.trim() == service_name);

    Ok(exists)
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
