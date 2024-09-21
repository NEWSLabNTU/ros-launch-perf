mod context;
mod execution;
mod launch_dump;
mod node_cmdline;
mod options;

use crate::{
    context::{
        prepare_load_node_contexts, prepare_node_contexts, LoadNodeContextSet, NodeContextSet,
    },
    execution::{
        build_container_groups, spawn_load_node_groups, spawn_load_nodes, spawn_node_containers,
        spawn_nodes,
    },
    launch_dump::{
        load_and_transform_node_records, load_launch_dump, ComposableNodeContainerRecord,
    },
    options::Options,
};
use clap::Parser;
use eyre::Context;
use futures::{future::BoxFuture, stream::FuturesUnordered, FutureExt, StreamExt};
use itertools::chain;
use rayon::prelude::*;
use std::{
    collections::HashSet,
    fs,
    io::{self, prelude::*},
    path::{Path, PathBuf},
    time::Duration,
};
use tokio::runtime::Runtime;
use tracing::{error, info};

fn main() -> eyre::Result<()> {
    // install global collector configured based on RUST_LOG env var.
    tracing_subscriber::fmt::init();

    let opts = Options::parse();

    if opts.print_shell {
        generate_shell(&opts)?;
    } else {
        Runtime::new()?.block_on(play(&opts))?;
    }

    Ok(())
}

/// Generate shell script from the launch record.
fn generate_shell(opts: &options::Options) -> eyre::Result<()> {
    let log_dir = prepare_log_dir(&opts.log_dir)?;
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
        .map(|request| request.to_shell());

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

/// Play the launch according to the launch record.
async fn play(opts: &options::Options) -> eyre::Result<()> {
    let load_node_delay = Duration::from_millis(opts.delay_load_node_millis);
    let launch_dump = load_launch_dump(&opts.input_file)?;

    // Prepare directories
    let log_dir = prepare_log_dir(&opts.log_dir)?;

    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", node_log_dir.display()))?;

    let load_node_log_dir = log_dir.join("load_node");
    fs::create_dir(&load_node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", load_node_log_dir.display()))?;

    // Build a table of composable node containers
    let container_names: HashSet<String> = launch_dump
        .container
        .par_iter()
        .map(|record| {
            let ComposableNodeContainerRecord { namespace, name } = record;
            format!("{namespace}/{name}")
        })
        .collect();

    // Prepare node execution contexts
    let NodeContextSet {
        container_contexts,
        pure_node_contexts,
    } = prepare_node_contexts(
        &launch_dump,
        &params_files_dir,
        &node_log_dir,
        &container_names,
    )?;

    // Prepare LoadNode request execution contexts
    let LoadNodeContextSet {
        nice_load_node_contexts,
        orphan_load_node_contexts,
    } = prepare_load_node_contexts(&launch_dump, &load_node_log_dir, &container_names)?;

    // Report the number of entities
    info!("nodes:\t{}", pure_node_contexts.len());
    info!("containers:\t{}", container_names.len());
    info!(
        "load node:\t{}",
        nice_load_node_contexts.len() + orphan_load_node_contexts.len()
    );
    info!("orphan load node:\t{}", orphan_load_node_contexts.len());

    // Build container groups
    let container_groups = build_container_groups(
        &container_names,
        container_contexts,
        nice_load_node_contexts,
    );

    // Spawn node containers in each node container group
    let (container_tasks, nice_load_node_groups) = spawn_node_containers(container_groups);
    info!("Spawned all node containers");

    // Spawn nodes
    let pure_node_tasks = spawn_nodes(pure_node_contexts);
    info!("Spawned all nodes");

    // Check if there are LoadNode requests
    let has_load_nodes =
        !(nice_load_node_groups.is_empty() && orphan_load_node_contexts.is_empty());

    let (nice_load_node_tasks, orphan_load_node_tasks) = if has_load_nodes {
        tokio::time::sleep(load_node_delay).await;

        // Spawn composable nodes having belonging containers.
        let nice_load_node_tasks = spawn_load_node_groups(nice_load_node_groups);

        // Spawn composable nodes which have no belonging containers.
        let orphan_load_node_tasks = spawn_load_nodes(orphan_load_node_contexts);

        info!("Spawned all composable nodes");
        (Some(nice_load_node_tasks), Some(orphan_load_node_tasks))
    } else {
        (None, None)
    };

    // Collect all process waiting tasks into one FuturesUnordered collection.
    let futures: FuturesUnordered<BoxFuture<eyre::Result<()>>> = chain!(
        container_tasks.into_iter().map(FutureExt::boxed),
        pure_node_tasks.into_iter().map(FutureExt::boxed),
        nice_load_node_tasks
            .into_iter()
            .flatten()
            .map(FutureExt::boxed),
        orphan_load_node_tasks
            .into_iter()
            .flatten()
            .map(FutureExt::boxed),
    )
    .collect();

    // Consume tasks.
    futures
        .for_each(|result| async move {
            if let Err(err) = result {
                error!("{err}");
            }
        })
        .await;

    Ok(())
}

pub fn prepare_log_dir(log_dir: &Path) -> eyre::Result<PathBuf> {
    // If the log_dir points to an existing file/dir, find a proper N
    // and rename it to "{log_dir}.N".
    if log_dir.exists() {
        let Some(file_name) = log_dir.file_name() else {
            todo!();
        };
        let Some(file_name) = file_name.to_str() else {
            todo!();
        };

        for nth in 1.. {
            let new_file_name = format!("{file_name}.{nth}");
            let new_log_dir = log_dir.with_file_name(new_file_name);

            if !new_log_dir.exists() {
                fs::rename(log_dir, &new_log_dir).wrap_err_with(|| {
                    format!(
                        "unable to move from {} to {}",
                        log_dir.display(),
                        new_log_dir.display()
                    )
                })?;
                break;
            }
        }
    }

    fs::create_dir(log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", log_dir.display()))?;

    Ok(log_dir.to_path_buf())
}
