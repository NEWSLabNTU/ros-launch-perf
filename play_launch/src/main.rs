mod container_readiness;
mod context;
mod execution;
mod launch_dump;
mod node_cmdline;
mod options;

use crate::{
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
};
use clap::Parser;
use eyre::{bail, Context};
use futures::{future::JoinAll, FutureExt};
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
        // Build the async runtime.
        let runtime = Runtime::new()?;

        // Run the whole playing task in the runtime.
        runtime.block_on(play(&opts))?;
    }

    Ok(())
}

/// Generate shell script from the launch record.
fn generate_shell(opts: &options::Options) -> eyre::Result<()> {
    let log_dir = create_log_dir(&opts.log_dir)?;
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
        .map(|request| request.to_shell(opts.standalone_composable_nodes));

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
    let launch_dump = load_launch_dump(&opts.input_file)?;

    // Prepare directories
    let log_dir = create_log_dir(&opts.log_dir)?;

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
            let NodeContainerRecord { namespace, name } = record;
            // Handle namespace ending with '/' to avoid double slashes
            if namespace.ends_with('/') {
                format!("{namespace}{name}")
            } else {
                format!("{namespace}/{name}")
            }
        })
        .collect();

    // Prepare node execution contexts
    let NodeContextClasses {
        container_contexts,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)?;

    // Prepare LoadNode request execution contexts
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;

    // Report the number of entities
    info!("nodes: {}", pure_node_contexts.len());

    // Spawn non-container nodes
    let non_container_node_tasks = spawn_nodes(pure_node_contexts)
        .into_iter()
        .map(|future| future.boxed());

    // Create composable node execution configuration
    let composable_node_config = ComposableNodeExecutionConfig {
        standalone_composable_nodes: opts.standalone_composable_nodes,
        load_orphan_composable_nodes: opts.load_orphan_composable_nodes,
        spawn_config: SpawnComposableNodeConfig {
            max_concurrent_spawn: opts.max_concurrent_load_node_spawn,
            max_attempts: opts.load_node_attempts,
            wait_timeout: Duration::from_millis(opts.load_node_timeout_millis),
        },
        load_node_delay: Duration::from_millis(opts.delay_load_node_millis),
        container_wait_config: if opts.skip_container_check {
            None
        } else {
            Some(crate::container_readiness::ContainerWaitConfig::new(
                opts.container_ready_timeout_secs,
                opts.container_poll_interval_ms,
            ))
        },
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

    // Poll on all waiting tasks and consume finished tasks
    // one-by-one.
    while !wait_futures.is_empty() {
        let (result, ix, _) = futures::future::select_all(&mut wait_futures).await;

        if let Err(err) = result {
            error!("{err}");
        }

        let future_to_discard = wait_futures.remove(ix);
        drop(future_to_discard);
    }

    Ok(())
}

/// Create a directory to store logging data.
///
/// If the directory already exists, move the directory to "log.N"
/// where N is a number.
pub fn create_log_dir(log_dir: &Path) -> eyre::Result<PathBuf> {
    // If the log_dir points to an existing file/dir, find a proper N
    // and rename it to "{log_dir}.N".
    if log_dir.exists() {
        let Some(file_name) = log_dir.file_name() else {
            bail!("unable to find the file name of {}", log_dir.display());
        };
        let Some(file_name) = file_name.to_str() else {
            bail!("the file name of {} is not Unicode", log_dir.display());
        };

        for nth in 1..=1000 {
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

        // Check if we exhausted all attempts
        if log_dir.exists() {
            bail!("unable to find available backup directory name after 1000 attempts");
        }
    }

    fs::create_dir(log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", log_dir.display()))?;

    Ok(log_dir.to_path_buf())
}
