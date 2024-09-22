use crate::{
    context::{ExecutionContext, LoadNodeContext, NodeContainerContext, NodeContext},
    launch_dump::LoadNodeRecord,
};
use eyre::WrapErr;
use futures::stream::StreamExt;
use std::{
    collections::{HashMap, HashSet},
    fs::{self, File},
    future::Future,
    io::prelude::*,
    path::{Path, PathBuf},
    process::{ExitStatus, Stdio},
    time::Duration,
};
use tokio::process::Command;
use tracing::{error, info, warn};

#[derive(Debug, Clone)]
pub struct SpawnComposableNodeConfig {
    pub max_concurrent_spawn: usize,
    pub max_attemps: usize,
    pub wait_timeout: Duration,
}

pub struct ContainerGroupContext<'a> {
    pub container_contexts: Vec<ExecutionContext>,
    pub load_node_contexts: Vec<LoadNodeContext<'a>>,
}

pub struct LoadNodeGroupContext<'a> {
    pub load_node_contexts: Vec<LoadNodeContext<'a>>,
}

pub fn build_container_groups<'a>(
    container_names: &HashSet<String>,
    container_contexts: Vec<NodeContainerContext<'_>>,
    nice_load_node_contexts: Vec<LoadNodeContext<'a>>,
) -> HashMap<String, ContainerGroupContext<'a>> {
    let mut container_groups: HashMap<String, _> = container_names
        .iter()
        .map(|container_name| {
            (
                container_name.to_string(),
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
            .get_mut(&context.record.target_container_name)
            .unwrap()
            .load_node_contexts
            .push(context);
    }

    container_groups
}

/// Spawn all node containers in each container group.
pub fn spawn_node_containers(
    container_groups: HashMap<String, ContainerGroupContext<'_>>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    HashMap<String, LoadNodeGroupContext>,
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

/// Run all composable nodes in each container group.
pub async fn run_load_node_groups(
    load_node_groups: HashMap<String, LoadNodeGroupContext<'_>>,
    config: &SpawnComposableNodeConfig,
) {
    let load_node_contexts: Vec<_> = load_node_groups
        .into_iter()
        .flat_map(|(_container_name, context)| context.load_node_contexts)
        .collect();

    run_load_nodes(load_node_contexts, config).await;
}

/// Spawn node processes.
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

/// Run composable nodes.
pub async fn run_load_nodes(
    load_node_contexts: Vec<LoadNodeContext<'_>>,
    config: &SpawnComposableNodeConfig,
) {
    let SpawnComposableNodeConfig {
        max_concurrent_spawn,
        max_attemps,
        wait_timeout,
    } = *config;

    futures::stream::iter(load_node_contexts)
        .map(|context| async move { run_load_node(&context, wait_timeout, max_attemps).await })
        .buffer_unordered(max_concurrent_spawn)
        .for_each(|()| async move {})
        .await;
}

async fn run_load_node(context: &LoadNodeContext<'_>, wait_timeout: Duration, max_attempts: usize) {
    let LoadNodeContext {
        log_name,
        output_dir,
        ..
    } = context;
    // dbg!(log_name);

    for round in 1..=max_attempts {
        match run_load_node_once(context, wait_timeout, round).await {
            Ok(true) => return,
            Ok(false) => {
                warn!("{log_name} fails to start. Retrying ,,, ({round}/{max_attempts})");
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

async fn run_load_node_once(
    context: &LoadNodeContext<'_>,
    timeout: Duration,
    round: usize,
) -> eyre::Result<bool> {
    let LoadNodeContext {
        log_name,
        output_dir,
        record,
    } = context;

    let mut command = prepare_load_node_command(record, output_dir, round)?;

    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("{log_name} is unable to be spawned"))?;

    if let Some(pid) = child.id() {
        let pid_path = output_dir.join(format!("pid.{round}"));
        let mut pid_file = File::create(pid_path)?;
        writeln!(pid_file, "{pid}")?;
    }

    let wait = child.wait();
    // let instant = std::time::Instant::now();
    let status = match tokio::time::timeout(timeout, wait).await {
        Ok(Ok(status)) => status,
        Ok(Err(err)) => {
            warn!("{log_name} fails with error: {err}");
            return Ok(false);
        }
        Err(_) => {
            warn!("{log_name} hangs more than {timeout:?}. Killing the process.");
            match child.kill().await {
                Ok(()) => {}
                Err(err) => {
                    error!("{log_name} is not able to be killed: {err}");
                }
            }
            return Ok(false);
        }
    };
    // dbg!(instant.elapsed());

    save_load_node_status(&status, output_dir, log_name)?;

    if !status.success() {
        warn!("{log_name} falis to start");
        return Ok(false);
    }

    Ok(true)
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

fn prepare_load_node_command(
    record: &LoadNodeRecord,
    output_dir: &Path,
    round: usize,
) -> eyre::Result<Command> {
    let command = record.to_command();
    let stdout_path = output_dir.join(format!("out.{round}"));
    let stderr_path = output_dir.join(format!("err.{round}"));
    let cmdline_path = output_dir.join(format!("cmdline.{round}"));

    fs::create_dir_all(&output_dir)?;

    {
        let mut cmdline_file = File::create(cmdline_path)?;
        cmdline_file.write_all(&record.to_shell())?;
    }

    let stdout_file = File::create(stdout_path)?;
    let stderr_file = File::create(&stderr_path)?;

    let mut command: Command = command.into();
    command.kill_on_drop(true);
    command.stdin(Stdio::null());
    command.stdout(stdout_file);
    command.stderr(stderr_file);

    eyre::Ok(command)
}
