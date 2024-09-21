use crate::context::{ExecutionContext, LoadNodeContext, NodeContainerContext, NodeContext};
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs::File,
    future::Future,
    io::prelude::*,
    path::Path,
    process::ExitStatus,
};
use tracing::{error, info};

pub struct ContainerGroupContext {
    pub container_contexts: Vec<ExecutionContext>,
    pub load_node_contexts: Vec<ExecutionContext>,
}

pub struct LoadNodeGroupContext {
    pub load_node_contexts: Vec<ExecutionContext>,
}

pub fn build_container_groups<'a>(
    container_names: &'a HashSet<String>,
    container_contexts: Vec<NodeContainerContext<'_>>,
    nice_load_node_contexts: Vec<LoadNodeContext<'_>>,
) -> HashMap<&'a str, ContainerGroupContext> {
    let mut container_groups: HashMap<&str, _> = container_names
        .iter()
        .map(|container_name| {
            (
                container_name.as_str(),
                ContainerGroupContext {
                    container_contexts: vec![],
                    load_node_contexts: vec![],
                },
            )
        })
        .collect();

    for context in container_contexts {
        container_groups
            .get_mut(context.node_container_name.as_str())
            .unwrap()
            .container_contexts
            .push(context.node_context.exec);
    }

    for context in nice_load_node_contexts {
        container_groups
            .get_mut(context.record.target_container_name.as_str())
            .unwrap()
            .load_node_contexts
            .push(context.exec);
    }

    container_groups
}

/// Spawn all node containers in each container group.
pub fn spawn_node_containers(
    container_groups: HashMap<&str, ContainerGroupContext>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    HashMap<&str, LoadNodeGroupContext>,
) {
    let (container_task_groups, load_node_groups): (Vec<_>, HashMap<_, _>) = container_groups
        .into_iter()
        .filter_map(|(container_name, group)| {
            let ContainerGroupContext {
                container_contexts,
                load_node_contexts,
            } = group;

            // Spawn all container nodes in the group.
            let container_tasks: Vec<_> = container_contexts
                .into_iter()
                .filter_map(|exec| {
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
                (container_name, LoadNodeGroupContext { load_node_contexts }),
            )
        })
        .unzip();

    let container_tasks = container_task_groups.into_iter().flatten().collect();

    (container_tasks, load_node_groups)
}

// Spawn all composable nodes in each container group.
pub fn spawn_load_node_groups(
    load_node_groups: HashMap<&str, LoadNodeGroupContext>,
) -> Vec<impl Future<Output = Result<(), eyre::Error>>> {
    load_node_groups
        .into_iter()
        .flat_map(|(_container_name, context)| context.load_node_contexts)
        .filter_map(|load_node_context| {
            let ExecutionContext {
                log_name,
                output_dir,
                mut command,
            } = load_node_context;

            let child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}",);
                    error!("Check {}", output_dir.display());
                    return None;
                }
            };
            let task = async move { wait_for_load_node(&log_name, &output_dir, child).await };

            Some(task)
        })
        .collect()
}

// Spawn node processes.
pub fn spawn_nodes(
    node_contexts: Vec<NodeContext<'_>>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    node_contexts
        .into_iter()
        .filter_map(|context| {
            let ExecutionContext {
                log_name,
                output_dir,
                mut command,
            } = context.exec;
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

pub fn spawn_load_nodes(
    load_node_contexts: Vec<LoadNodeContext<'_>>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    load_node_contexts
        .into_iter()
        .filter_map(|context| {
            let ExecutionContext {
                log_name,
                output_dir,
                mut command,
            } = context.exec;
            let child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}",);
                    error!("Check {}", output_dir.display());
                    return None;
                }
            };
            let task = async move { wait_for_load_node(&log_name, &output_dir, child).await };
            Some(task)
        })
        .collect()
}

pub async fn wait_for_node(
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

pub async fn wait_for_load_node(
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
    save_load_node_status(&status, output_dir, log_name)?;
    eyre::Ok(())
}

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

fn save_load_node_status(
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
        // eprintln!("[{log_name}] finishes")
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
