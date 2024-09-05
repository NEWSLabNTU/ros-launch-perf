mod launch_dump;
mod options;

use crate::options::Options;
use clap::Parser;
use futures::{stream::FuturesUnordered, try_join, TryStreamExt};
use itertools::Itertools;
use launch_dump::{LaunchDump, ProcessKind};
use rayon::prelude::*;
use ros_cmdlines::CommandLine;
use std::{
    borrow::Borrow,
    fs,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
    time::Duration,
};
use tokio::runtime::Runtime;

fn main() -> eyre::Result<()> {
    let opts = Options::parse();

    match opts {
        Options::GenerateScript(opts) => {
            generate_shell(&opts)?;
        }
        Options::Run(opts) => {
            Runtime::new()?.block_on(run(&opts))?;
        }
    }

    Ok(())
}

fn generate_shell(opts: &options::GenerateScript) -> eyre::Result<()> {
    let new_params_dir = match &opts.copy_params_dir {
        Some(dir) => Some(dir.canonicalize()?),
        None => None,
    };

    let proc_info = load_and_transofmr_launch_dump(&opts.input_file, new_params_dir.as_deref())?;

    let shells: Result<Vec<_>, _> = proc_info
        .par_iter()
        .map(|info| {
            // Generate shell script
            let shell = info.cmdline.to_shell(false);
            eyre::Ok(shell)
        })
        .collect();

    println!("#!/bin/sh");
    for shell in shells? {
        println!("{shell}");
    }

    Ok(())
}

async fn run(opts: &options::Run) -> eyre::Result<()> {
    let new_params_dir = match &opts.copy_params_dir {
        Some(dir) => Some(dir.canonicalize()?),
        None => None,
    };

    let proc_info = load_and_transofmr_launch_dump(&opts.input_file, new_params_dir.as_deref())?;

    let commands: Result<Vec<_>, _> = proc_info
        .par_iter()
        .map(|info| {
            // Create a command object
            let mut command: tokio::process::Command = info.cmdline.to_command(false).into();
            command.kill_on_drop(true);

            eyre::Ok((info.kind, command))
        })
        .collect();

    // Unpack the outer Result.
    let commands: Vec<_> = commands?;

    // Separate containers and nodes.
    let (containers, nodes): (Vec<_>, Vec<_>) = commands
        .into_par_iter()
        .partition(|(kind, _)| *kind == ProcessKind::ComposableNodeContainer);

    // Create futures for execution.
    let container_tasks: FuturesUnordered<_> = containers
        .into_iter()
        .map(|(_, mut command)| async move {
            command.status().await?;
            eyre::Ok(())
        })
        .collect();
    let node_tasks: FuturesUnordered<_> = nodes
        .into_iter()
        .map(|(_, mut command)| async move {
            command.status().await?;
            eyre::Ok(())
        })
        .collect();

    // Run the tasks and wait for completion.
    try_join!(
        container_tasks.try_for_each(|_| async move { Ok(()) }),
        async move {
            // Wait for a short period to make sure the containers are
            // ready.
            tokio::time::sleep(Duration::from_millis(100)).await;

            node_tasks.try_for_each(|_| async move { Ok(()) }).await?;
            eyre::Ok(())
        },
    )?;

    Ok(())
}

fn load_and_transofmr_launch_dump(
    dump_file: &Path,
    new_params_dir: Option<&Path>,
) -> eyre::Result<Vec<ProcessInfo>> {
    // Load the launch dump file.
    let LaunchDump { process, file_data } = {
        let reader = BufReader::new(File::open(dump_file)?);
        serde_json::from_reader(reader)?
    };

    let info: Result<Vec<_>, _> = process
        .par_iter()
        .map(|process| {
            let mut cmdline = CommandLine::from_cmdline(&process.cmdline)?;

            if let Some(params_dir) = &new_params_dir {
                // Copy log_config_file to params dir.
                if let Some(src_path) = &cmdline.log_config_file {
                    let Some(data) = file_data.get(src_path) else {
                        todo!();
                    };
                    cmdline.log_config_file = Some(copy_file(src_path, params_dir, data)?);
                }

                // Copy params_file to params dir.
                cmdline.params_files = cmdline
                    .params_files
                    .iter()
                    .map(|src_path| {
                        let Some(data) = file_data.get(src_path) else {
                            todo!();
                        };
                        let tgt_path = copy_file(src_path, params_dir, data)?;
                        eyre::Ok(tgt_path)
                    })
                    .try_collect()?;
            }

            eyre::Ok(ProcessInfo {
                kind: process.kind,
                cmdline,
            })
        })
        .collect();

    info
}

#[derive(Debug, Clone)]
struct ProcessInfo {
    pub kind: ProcessKind,
    pub cmdline: CommandLine,
}

/// An utility function to copy saved data content to the new directory.
fn copy_file(src_path: &Path, tgt_dir: &Path, data: &str) -> eyre::Result<PathBuf> {
    let file_name = url_escape::encode_component(src_path.to_str().unwrap());
    let file_name: &str = file_name.borrow();
    let tgt_path = tgt_dir.join(file_name);
    fs::write(&tgt_path, data)?;
    eyre::Ok(tgt_path)
}
