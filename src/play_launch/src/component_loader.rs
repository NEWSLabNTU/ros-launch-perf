// Component loader module - loads composable nodes via ROS service calls
//
// This module provides a background thread that handles loading composable nodes
// into containers using the composition_interfaces/srv/LoadNode service.
//
// The approach uses direct ROS service calls via rclrs instead of spawning
// `ros2 component load` subprocesses, which eliminates Python startup overhead
// and enables better parallelization.

use eyre::{Context as _, Result};
use std::{collections::HashMap, time::Duration};
use tokio::sync::{mpsc, oneshot};
use tracing::{debug, error, info, warn};

/// Response from loading a composable node
#[derive(Clone, Debug, PartialEq)]
pub struct LoadNodeResponse {
    pub success: bool,
    pub error_message: String,
    pub full_node_name: String,
    pub unique_id: u64,
}

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
    #[allow(clippy::too_many_arguments)]
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

    // Cache service clients for each container
    let mut clients: HashMap<String, rclrs::Client<composition_interfaces::srv::LoadNode>> =
        HashMap::new();

    while let Some(request) = request_rx.blocking_recv() {
        debug!(
            "Loading node {}/{} into container {}",
            request.package_name, request.plugin_name, request.container_name
        );

        // Get or create service client for this container
        let service_name = format!("{}/_container/load_node", request.container_name);
        let client = clients.entry(service_name.clone()).or_insert_with(|| {
            debug!("Creating service client for {}", service_name);
            node.create_client::<composition_interfaces::srv::LoadNode>(&service_name)
                .expect("Failed to create service client")
        });

        // Call the service
        let result = call_component_load_service(
            client,
            &request.package_name,
            &request.plugin_name,
            &request.node_name,
            &request.node_namespace,
            &request.remap_rules,
            &request.parameters,
            &request.extra_args,
            request.timeout,
        );

        let _ = request.response_tx.send(result);
    }

    info!("Component loader shutting down");
    Ok(())
}

/// Call LoadNode service using rclrs
///
/// This replaces the subprocess-based approach with direct service calls
#[allow(clippy::too_many_arguments)]
fn call_component_load_service(
    client: &rclrs::Client<composition_interfaces::srv::LoadNode>,
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
        "Calling LoadNode service for {}/{} (node_name: {}, namespace: {})",
        package, plugin, node_name, namespace
    );

    // Build the service request
    let request = composition_interfaces::srv::LoadNode_Request {
        package_name: package.to_string(),
        plugin_name: plugin.to_string(),
        node_name: node_name.to_string(),
        node_namespace: namespace.to_string(),
        log_level: 0, // Default log level
        remap_rules: remap_rules.to_vec(),
        parameters: convert_parameters_to_ros(parameters)?,
        extra_arguments: convert_parameters_to_ros(extra_args)?,
    };

    // Call the service
    debug!("Sending service request...");
    let response_promise: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> = client
        .call(&request)
        .wrap_err_with(|| format!("Failed to call LoadNode service for {}/{}", package, plugin))?;

    // Wait for response with timeout
    let runtime = tokio::runtime::Runtime::new()
        .context("Failed to create tokio runtime for service call")?;

    let response_result =
        runtime.block_on(async { tokio::time::timeout(timeout, response_promise).await });

    match response_result {
        Ok(Ok(response)) => {
            debug!(
                "Received response: success={}, full_node_name={}, unique_id={}",
                response.success, response.full_node_name, response.unique_id
            );

            Ok(LoadNodeResponse {
                success: response.success,
                error_message: response.error_message.clone(),
                full_node_name: response.full_node_name.clone(),
                unique_id: response.unique_id,
            })
        }
        Ok(Err(e)) => {
            error!("Service call failed: {:#}", e);
            Ok(LoadNodeResponse {
                success: false,
                error_message: format!("Service call failed: {:#}", e),
                full_node_name: String::new(),
                unique_id: 0,
            })
        }
        Err(_) => {
            warn!("Service call timed out after {:?}", timeout);
            Ok(LoadNodeResponse {
                success: false,
                error_message: format!("Timeout after {:?}", timeout),
                full_node_name: String::new(),
                unique_id: 0,
            })
        }
    }
}

/// Convert string parameters to ROS Parameter messages
fn convert_parameters_to_ros(
    params: &[(String, String)],
) -> Result<Vec<rcl_interfaces::msg::Parameter>> {
    params
        .iter()
        .map(|(name, value)| {
            Ok(rcl_interfaces::msg::Parameter {
                name: name.clone(),
                value: parse_parameter_value(value)?,
            })
        })
        .collect()
}

/// Parse a parameter value string into a ParameterValue
fn parse_parameter_value(value: &str) -> Result<rcl_interfaces::msg::ParameterValue> {
    use rcl_interfaces::msg::ParameterType;

    // Try to parse as different types
    // Boolean
    if value == "true" || value == "True" {
        return Ok(rcl_interfaces::msg::ParameterValue {
            type_: ParameterType::PARAMETER_BOOL as u8,
            bool_value: true,
            integer_value: 0,
            double_value: 0.0,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        });
    }
    if value == "false" || value == "False" {
        return Ok(rcl_interfaces::msg::ParameterValue {
            type_: ParameterType::PARAMETER_BOOL as u8,
            bool_value: false,
            integer_value: 0,
            double_value: 0.0,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        });
    }

    // Integer
    if let Ok(int_val) = value.parse::<i64>() {
        return Ok(rcl_interfaces::msg::ParameterValue {
            type_: ParameterType::PARAMETER_INTEGER as u8,
            bool_value: false,
            integer_value: int_val,
            double_value: 0.0,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        });
    }

    // Double
    if let Ok(double_val) = value.parse::<f64>() {
        return Ok(rcl_interfaces::msg::ParameterValue {
            type_: ParameterType::PARAMETER_DOUBLE as u8,
            bool_value: false,
            integer_value: 0,
            double_value: double_val,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        });
    }

    // Default to string
    Ok(rcl_interfaces::msg::ParameterValue {
        type_: ParameterType::PARAMETER_STRING as u8,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: value.to_string(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    })
}
