use crate::context::{ComposableNodeContext, ExecutionContext, NodeContainerContext, NodeContext};
use eyre::WrapErr;
use futures::{
    future::{BoxFuture, FutureExt},
    stream::StreamExt,
};
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs::File,
    future::Future,
    io::prelude::*,
    path::Path,
    process::ExitStatus,
    time::Duration,
};
use tracing::{debug, error, info, warn};

/// The options to spawn a composable node loading process.
#[derive(Debug, Clone, Copy)]
pub struct SpawnComposableNodeConfig {
    pub max_concurrent_spawn: usize,
    pub max_attempts: usize,
    pub wait_timeout: Duration,
}

/// Configuration for executing composable nodes.
#[derive(Debug, Clone)]
pub struct ComposableNodeExecutionConfig {
    pub standalone_composable_nodes: bool,
    pub load_orphan_composable_nodes: bool,
    pub spawn_config: SpawnComposableNodeConfig,
    pub load_node_delay: Duration,
    pub container_wait_config: Option<crate::container_readiness::ContainerWaitConfig>,
}

/// The set of node containers and composable nodes belong to the same
/// container name.
struct NodeContainerGroup {
    pub container_contexts: Vec<NodeContext>,
    pub composable_node_contexts: Vec<ComposableNodeContext>,
}

/// The set of composable nodes belong to the same container name.
pub struct ComposableNodeGroup {
    pub composable_node_contexts: Vec<ComposableNodeContext>,
}

/// The set of tasks waiting for the completion of node container and
/// composable node loading/execution processes.
pub enum ComposableNodeTasks {
    Standalone {
        wait_composable_node_tasks: Vec<BoxFuture<'static, eyre::Result<()>>>,
    },
    Container {
        wait_container_tasks: Vec<BoxFuture<'static, eyre::Result<()>>>,
        load_nice_composable_nodes_task: BoxFuture<'static, ()>,
        load_orphan_composable_nodes_task: Option<BoxFuture<'static, ()>>,
    },
}

/// Spawn ROS node processes.
pub fn spawn_nodes(node_contexts: Vec<NodeContext>) -> Vec<impl Future<Output = eyre::Result<()>>> {
    node_contexts
        .into_iter()
        .filter_map(|context| {
            let exec = match context.to_exec_context() {
                Ok(exec) => exec,
                Err(err) => {
                    error!("Unable to prepare execution for node: {err}",);
                    error!("which is caused by the node: {:#?}", context.record);
                    return None;
                }
            };
            let ExecutionContext {
                log_name,
                output_dir,
                mut command,
            } = exec;
            let child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}",);
                    error!("Check {}", output_dir.display());
                    return None;
                }
            };
            let task = async move { wait_for_node(&log_name, &output_dir, child).await };
            Some(task)
        })
        .collect()
}

/// Load composable node into containers or execute them in standalone
/// containers.
pub fn spawn_or_load_composable_nodes(
    container_contexts: Vec<NodeContainerContext>,
    load_node_contexts: Vec<ComposableNodeContext>,
    container_names: &HashSet<String>,
    config: ComposableNodeExecutionConfig,
) -> ComposableNodeTasks {
    if config.standalone_composable_nodes {
        info!("standalone composable node: {}", load_node_contexts.len());

        let standalone_composable_node_tasks =
            spawn_standalone_composable_nodes(load_node_contexts);

        ComposableNodeTasks::Standalone {
            wait_composable_node_tasks: standalone_composable_node_tasks
                .into_iter()
                .map(|future| future.boxed())
                .collect(),
        }
    } else {
        // Separate orphan and non-orphan composable nodes
        let (nice_load_node_contexts, orphan_load_node_contexts): (Vec<_>, Vec<_>) =
            load_node_contexts.into_par_iter().partition_map(|context| {
                use rayon::iter::Either;

                if container_names.contains(&context.record.target_container_name) {
                    Either::Left(context)
                } else {
                    Either::Right(context)
                }
            });

        info!("node containers: {}", container_names.len());
        info!(
            "loaded composable node: {}",
            nice_load_node_contexts.len() + orphan_load_node_contexts.len()
        );

        // Spawn node containers and prepare composable node
        // loading tasks
        let (container_tasks, load_nice_composable_nodes_task) =
            spawn_node_containers_and_load_composable_nodes(
                container_names,
                container_contexts,
                nice_load_node_contexts,
                config.spawn_config,
                config.load_node_delay,
                config.container_wait_config.clone(),
            );

        let container_tasks: Vec<_> = container_tasks
            .into_iter()
            .map(|future| future.boxed())
            .collect();

        // Optionally load orphan composable nodes
        let load_orphan_composable_nodes_task = if orphan_load_node_contexts.is_empty() {
            None
        } else if config.load_orphan_composable_nodes {
            info!(
                "orphan composable node: {}",
                orphan_load_node_contexts.len()
            );
            let task = run_load_composable_nodes(orphan_load_node_contexts, config.spawn_config);
            Some(task.boxed())
        } else {
            warn!(
                "{} orphan composable nodes are not loaded",
                orphan_load_node_contexts.len()
            );
            None
        };

        ComposableNodeTasks::Container {
            wait_container_tasks: container_tasks,
            load_nice_composable_nodes_task: load_nice_composable_nodes_task.boxed(),
            load_orphan_composable_nodes_task,
        }
    }
}

/// Classify the container and composable node contexts into groups by
/// their container name.
fn build_container_groups(
    container_names: &HashSet<String>,
    container_contexts: Vec<NodeContainerContext>,
    nice_load_node_contexts: Vec<ComposableNodeContext>,
) -> HashMap<String, NodeContainerGroup> {
    let mut container_groups: HashMap<String, _> = container_names
        .iter()
        .map(|container_name| {
            (
                container_name.to_string(),
                NodeContainerGroup {
                    container_contexts: vec![],
                    composable_node_contexts: vec![],
                },
            )
        })
        .collect();

    for context in container_contexts {
        container_groups
            .get_mut(context.node_container_name.as_str())
            .expect("unable to find the container for the LoadNode request")
            .container_contexts
            .push(context.node_context);
    }

    for context in nice_load_node_contexts {
        container_groups
            .get_mut(&context.record.target_container_name)
            .expect("unable to find the container for the composable node")
            .composable_node_contexts
            .push(context);
    }

    container_groups
}

/// Spawn all node containers in each container group.
fn spawn_node_containers(
    container_groups: HashMap<String, NodeContainerGroup>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    HashMap<String, ComposableNodeGroup>,
) {
    let (container_task_groups, load_node_groups): (Vec<_>, HashMap<_, _>) = container_groups
        .into_iter()
        .filter_map(|(container_name, group)| {
            let NodeContainerGroup {
                container_contexts,
                composable_node_contexts: load_node_contexts,
            } = group;

            // Spawn all container nodes in the group.
            let container_tasks: Vec<_> = container_contexts
                .into_iter()
                .filter_map(|context| {
                    let exec = match context.to_exec_context() {
                        Ok(exec) => exec,
                        Err(err) => {
                            error!("Unable to prepare execution for node: {err}",);
                            error!("which is caused by the node: {:#?}", context.record);
                            return None;
                        }
                    };
                    let ExecutionContext {
                        log_name,
                        output_dir,
                        mut command,
                    } = exec;

                    let child = match command.spawn() {
                        Ok(child) => child,
                        Err(err) => {
                            error!("{log_name} is unable to start: {err}",);
                            error!("Check {}", output_dir.display());
                            return None;
                        }
                    };
                    let task = async move { wait_for_node(&log_name, &output_dir, child).await };

                    Some(task)
                })
                .collect();

            // If there is no container node spawned successfully,
            // skip later load node tasks.
            if container_tasks.is_empty() {
                return None;
            }

            Some((container_name, container_tasks, load_node_contexts))
        })
        .map(|(container_name, container_tasks, load_node_contexts)| {
            (
                container_tasks,
                (
                    container_name,
                    ComposableNodeGroup {
                        composable_node_contexts: load_node_contexts,
                    },
                ),
            )
        })
        .unzip();

    let container_tasks = container_task_groups.into_iter().flatten().collect();

    (container_tasks, load_node_groups)
}

/// Run all composable nodes in each container group.
async fn run_load_composable_node_groups(
    load_node_groups: HashMap<String, ComposableNodeGroup>,
    config: SpawnComposableNodeConfig,
) {
    let load_node_contexts: Vec<_> = load_node_groups
        .into_iter()
        .flat_map(|(_container_name, context)| context.composable_node_contexts)
        .collect();

    run_load_composable_nodes(load_node_contexts, config).await;
}

/// Spawn node container processes and load all composable nodes to
/// them.
fn spawn_node_containers_and_load_composable_nodes(
    container_names: &HashSet<String>,
    container_contexts: Vec<NodeContainerContext>,
    load_node_contexts: Vec<ComposableNodeContext>,
    config: SpawnComposableNodeConfig,
    load_node_delay: Duration,
    container_wait_config: Option<crate::container_readiness::ContainerWaitConfig>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    impl Future<Output = ()>,
) {
    // Build container groups
    let container_groups =
        build_container_groups(container_names, container_contexts, load_node_contexts);

    // Spawn node containers in each node container group
    let (container_tasks, nice_load_node_groups) = spawn_node_containers(container_groups);

    let container_names_vec: Vec<String> = container_names.iter().cloned().collect();

    let load_composable_nodes_task = async move {
        if nice_load_node_groups.is_empty() {
            return;
        }

        // Wait for containers to be ready
        if let Some(wait_config) = container_wait_config {
            // New behavior: wait for service availability
            info!("Waiting for containers to be ready...");
            if let Err(e) = crate::container_readiness::wait_for_containers_ready(
                &container_names_vec,
                &wait_config,
            )
            .await
            {
                error!("Error waiting for containers: {}", e);
            }
        } else {
            // Legacy behavior: fixed delay
            info!(
                "Using fixed delay of {:?} before loading composable nodes",
                load_node_delay
            );
            tokio::time::sleep(load_node_delay).await;
        }

        // Spawn composable nodes having belonging containers.
        run_load_composable_node_groups(nice_load_node_groups, config).await;
    };

    (container_tasks, load_composable_nodes_task)
}

/// Spawn standalone composable nodes.
fn spawn_standalone_composable_nodes(
    load_node_contexts: Vec<ComposableNodeContext>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    load_node_contexts
        .into_iter()
        .filter_map(|context| {
            let ComposableNodeContext {
                log_name,
                output_dir,
                ..
            } = &context;
            let mut command = match context.to_standalone_node_command() {
                Ok(command) => command,
                Err(err) => {
                    error!("{log_name} fails to generate command: {err}",);
                    return None;
                }
            };

            let child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}",);
                    error!("Check {}", output_dir.display());
                    return None;
                }
            };
            let task = async move {
                let ComposableNodeContext {
                    log_name,
                    output_dir,
                    ..
                } = &context;
                wait_for_node(log_name, output_dir, child).await
            };
            Some(task)
        })
        .collect()
}

/// Load a set of composable nodes.
async fn run_load_composable_nodes(
    load_node_contexts: Vec<ComposableNodeContext>,
    config: SpawnComposableNodeConfig,
) {
    let SpawnComposableNodeConfig {
        max_concurrent_spawn,
        max_attempts,
        wait_timeout,
    } = config;

    let total_count = load_node_contexts.len();

    let mut futures = futures::stream::iter(load_node_contexts.into_iter())
        .map(|context| async move {
            run_load_composable_node(&context, wait_timeout, max_attempts).await
        })
        .buffer_unordered(max_concurrent_spawn)
        .zip(futures::stream::iter(0..));

    while let Some(((), nth)) = futures.next().await {
        if (nth + 1) % 10 == 0 {
            info!(
                "Done loading {} out of {total_count} composable nodes.",
                nth + 1
            );
        }
    }
}

/// Load one composable node up to a max number of failing attempts.
async fn run_load_composable_node(
    context: &ComposableNodeContext,
    wait_timeout: Duration,
    max_attempts: usize,
) {
    let ComposableNodeContext {
        log_name,
        output_dir,
        ..
    } = context;
    debug!("{log_name} is loading");

    for round in 1..=max_attempts {
        match run_load_composable_node_once(context, wait_timeout, round).await {
            Ok(true) => return,
            Ok(false) => {
                warn!("{log_name} fails to start. Retrying... ({round}/{max_attempts})");
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            Err(err) => {
                error!("{log_name} fails to due error: {err}");
                break;
            }
        }
    }

    error!("{log_name} is unable to start");
    error!("Check {}", output_dir.display());
}

/// Load one composable node once.
async fn run_load_composable_node_once(
    context: &ComposableNodeContext,
    timeout: Duration,
    round: usize,
) -> eyre::Result<bool> {
    let ComposableNodeContext {
        log_name,
        output_dir,
        ..
    } = context;

    let mut command = context.to_load_node_command(round)?;

    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("{log_name} is unable to be spawned"))?;

    if let Some(pid) = child.id() {
        let pid_path = output_dir.join(format!("pid.{round}"));
        let mut pid_file = File::create(pid_path)?;
        writeln!(pid_file, "{pid}")?;
    }

    let wait = child.wait();
    let status = match tokio::time::timeout(timeout, wait).await {
        Ok(Ok(status)) => status,
        Ok(Err(err)) => {
            warn!("{log_name} fails with error: {err}");
            return Ok(false);
        }
        Err(_) => {
            warn!("{log_name} hangs more than {timeout:?}. Try killing the process.");
            match child.kill().await {
                Ok(()) => {}
                Err(err) => {
                    error!("{log_name} is not able to be killed: {err}");
                }
            }
            return Ok(false);
        }
    };

    save_composable_node_status(&status, output_dir, log_name)?;

    if !status.success() {
        warn!("{log_name} fails to start");
        return Ok(false);
    }

    Ok(true)
}

/// Wait for the completion of a node process.
async fn wait_for_node(
    log_name: &str,
    output_dir: &Path,
    mut child: tokio::process::Child,
) -> eyre::Result<()> {
    if let Some(pid) = child.id() {
        let pid_path = output_dir.join("pid");
        let mut pid_file = File::create(pid_path)?;
        writeln!(pid_file, "{pid}")?;
    }

    let status = child.wait().await?;
    save_node_status(&status, output_dir, log_name)?;
    eyre::Ok(())
}

/// Save the status code for a completed node process.
fn save_node_status(status: &ExitStatus, output_dir: &Path, log_name: &str) -> eyre::Result<()> {
    let status_path = output_dir.join("status");

    // Save status code
    {
        let mut status_file = File::create(status_path)?;
        if let Some(code) = status.code() {
            writeln!(status_file, "{code}",)?;
        } else {
            writeln!(status_file, "[none]")?;
        }
    }

    // Print status to the terminal
    if status.success() {
        info!("[{log_name}] finishes")
    } else {
        match status.code() {
            Some(code) => {
                error!("{log_name} fails with code {code}.",);
                error!("Check {}", output_dir.display());
            }
            None => {
                error!("{log_name} fails.",);
                error!("Check {}", output_dir.display());
            }
        }
    }

    eyre::Ok(())
}

/// Save the status code for a completed composable node
/// loading/execution process.
fn save_composable_node_status(
    status: &ExitStatus,
    output_dir: &Path,
    log_name: &str,
) -> eyre::Result<()> {
    let status_path = output_dir.join("status");

    // Save status code
    {
        let mut status_file = File::create(status_path)?;
        if let Some(code) = status.code() {
            writeln!(status_file, "{code}",)?;
        } else {
            writeln!(status_file, "[none]")?;
        }
    }

    // Print status to the terminal
    if status.success() {
        debug!("{log_name} loading finishes successfully");
    } else {
        match status.code() {
            Some(code) => {
                error!("{log_name} fails with code {code}.",);
                error!("Check {}", output_dir.display());
            }
            None => {
                error!("[{log_name}] fails.",);
                error!("Check {}", output_dir.display());
            }
        }
    }

    eyre::Ok(())
}
