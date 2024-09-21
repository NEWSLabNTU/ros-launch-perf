use crate::context::{ExecutionContext, LoadNodeContext, NodeContainerContext};
use futures::stream::{FuturesUnordered, StreamExt};
use std::{
    collections::{HashMap, HashSet},
    fs::File,
    future::Future,
    io::prelude::*,
    path::Path,
    process::ExitStatus,
};
use tracing::{error, info};

pub struct ContainerGroup<CF, LF> {
    pub container_tasks: Vec<CF>,
    pub load_node_tasks: Vec<LF>,
}

pub fn build_container_groups<'a>(
    container_names: &'a HashSet<String>,
    container_contexts: Vec<NodeContainerContext<'_>>,
    nice_load_node_contexts: Vec<LoadNodeContext<'_>>,
) -> HashMap<&'a str, ContainerGroup<ExecutionContext, ExecutionContext>> {
    let mut container_groups: HashMap<&str, _> = container_names
        .iter()
        .map(|container_name| {
            (
                container_name.as_str(),
                ContainerGroup {
                    container_tasks: vec![],
                    load_node_tasks: vec![],
                },
            )
        })
        .collect();

    for context in container_contexts {
        container_groups
            .get_mut(context.node_container_name.as_str())
            .unwrap()
            .container_tasks
            .push(context.node_context.exec);
    }

    for context in nice_load_node_contexts {
        container_groups
            .get_mut(context.record.target_container_name.as_str())
            .unwrap()
            .load_node_tasks
            .push(context.exec);
    }

    container_groups
}

pub async fn consume_futures<Fut>(futures: FuturesUnordered<Fut>)
where
    Fut: Future<Output = eyre::Result<()>>,
{
    futures
        .for_each(|result| async move {
            if let Err(err) = result {
                error!("{err}");
            }
        })
        .await;
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
