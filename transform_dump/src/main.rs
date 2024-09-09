mod launch_dump;
mod options;

use crate::options::Options;
use clap::Parser;
use futures::{stream::FuturesUnordered, try_join, TryStreamExt};
use itertools::Itertools;
use launch_dump::LaunchDump;
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
        Options::Play(opts) => {
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

    let launch_dump = load_launch_dump(&opts.input_file)?;

    let process_records =
        load_and_transform_process_records(&launch_dump, new_params_dir.as_deref())?;
    let process_shells = process_records
        .par_iter()
        .map(|info| info.cmdline.to_shell(false));

    let load_node_shells = launch_dump
        .load_node
        .par_iter()
        .map(|request| request.to_shell());

    let mut shells: Vec<_> = process_shells.chain(load_node_shells).collect();
    shells.par_sort_unstable();

    println!("#!/bin/sh");
    for shell in shells {
        println!("{shell}");
    }

    Ok(())
}

async fn run(opts: &options::Play) -> eyre::Result<()> {
    let new_params_dir = match &opts.copy_params_dir {
        Some(dir) => Some(dir.canonicalize()?),
        None => None,
    };

    let launch_dump = load_launch_dump(&opts.input_file)?;

    let process_records =
        load_and_transform_process_records(&launch_dump, new_params_dir.as_deref())?;
    let process_commands: Result<Vec<_>, _> = process_records
        .par_iter()
        .map(|info| {
            // Create a command object
            let mut command: tokio::process::Command = info.cmdline.to_command(false).into();
            command.kill_on_drop(true);

            eyre::Ok(command)
        })
        .collect();

    // Unpack the outer Result.
    let process_commands: Vec<_> = process_commands?;

    let process_futures: FuturesUnordered<_> = process_commands
        .into_iter()
        .map(|mut command| command.status())
        .collect();

    let load_node_futures: FuturesUnordered<_> = launch_dump
        .load_node
        .iter()
        .map(|request| {
            eprintln!("{}", request.to_shell());
            request.to_command()
        })
        .map(|command| {
            let mut command: tokio::process::Command = command.into();
            command.kill_on_drop(true);
            command
        })
        .map(|mut command| command.status())
        .collect();

    try_join!(
        async move {
            process_futures
                .try_for_each(|_| futures::future::ok(()))
                .await?;
            eyre::Ok(())
        },
        async move {
            // Postpone the load node commands to wait for containers
            // to be ready.
            tokio::time::sleep(Duration::from_millis(3000)).await;

            load_node_futures
                .try_for_each(|_| futures::future::ok(()))
                .await?;
            eyre::Ok(())
        }
    )?;

    Ok(())
}

fn load_launch_dump(dump_file: &Path) -> eyre::Result<LaunchDump> {
    let reader = BufReader::new(File::open(dump_file)?);
    let launch_dump = serde_json::from_reader(reader)?;
    Ok(launch_dump)
}

fn load_and_transform_process_records(
    launch_dump: &LaunchDump,
    new_params_dir: Option<&Path>,
) -> eyre::Result<Vec<Execution>> {
    let LaunchDump {
        node, file_data, ..
    } = launch_dump;

    let info: Result<Vec<_>, _> = node
        .par_iter()
        .map(|process| {
            let mut cmdline = CommandLine::from_cmdline(&process.cmd)?;

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

            eyre::Ok(Execution { cmdline })
        })
        .collect();

    info
}

#[derive(Debug, Clone)]
struct Execution {
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
