mod launch_dump;
mod node_cmdline;
mod options;

use crate::{launch_dump::LoadNodeRecord, options::Options};
use clap::Parser;
use eyre::{bail, ensure, Context};
use futures::{join, stream::FuturesUnordered, StreamExt};
use itertools::Itertools;
use launch_dump::{LaunchDump, NodeRecord};
use node_cmdline::NodeCommandLine;
use rayon::{iter::Either, prelude::*};
use std::{
    collections::HashMap,
    fs,
    fs::File,
    io::{self, prelude::*, BufReader},
    path::{Path, PathBuf, MAIN_SEPARATOR},
    process::{ExitStatus, Stdio},
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

async fn run(opts: &options::Play) -> eyre::Result<()> {
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

    // Prepare node execution contexts
    let node_cmdlines = load_and_transform_node_records(&launch_dump, &params_files_dir)?;
    let node_tasks: Result<Vec<_>, _> = node_cmdlines
        .into_par_iter()
        .map(|(record, cmdline)| {
            let Some(exec_name) = &record.exec_name else {
                bail!(r#"expect the "exec_name" field but not found"#);
            };
            let Some(package) = &record.package else {
                bail!(r#"expect the "package" field but not found"#);
            };

            let output_dir = node_log_dir.join(package).join(exec_name);
            ensure!(output_dir.is_relative());
            let stdout_path = output_dir.join("out");
            let stderr_path = output_dir.join("err");
            let cmdline_path = output_dir.join("cmdline");

            fs::create_dir_all(&output_dir)?;

            {
                let mut cmdline_file = File::create(cmdline_path)?;
                cmdline_file.write_all(&cmdline.to_shell(false))?;
            }

            let stdout_file = File::create(stdout_path)?;
            let stderr_file = File::create(&stderr_path)?;

            let mut command: tokio::process::Command = cmdline.to_command(false).into();
            command.kill_on_drop(true);
            command.stdin(Stdio::null());
            command.stdout(stdout_file);
            command.stderr(stderr_file);

            let task = async move {
                let mut child = command.spawn()?;
                let status = child.wait().await?;
                let log_name = format!("NODE {package} {exec_name}");
                save_status(&status, &output_dir, &log_name)?;

                eyre::Ok(())
            };

            eyre::Ok(task)
        })
        .collect();
    let node_tasks = node_tasks?;

    // Prepare LoadNode request execution contexts
    let load_node_records = &launch_dump.load_node;
    let load_node_tasks: Result<Vec<_>, _> = load_node_records
        .par_iter()
        .map(|record| {
            let command = record.to_command();
            let LoadNodeRecord {
                package,
                plugin,
                target_container_name,
                ..
            } = record;
            let output_dir = load_node_log_dir
                .join(target_container_name.replace("/", "!"))
                .join(package)
                .join(plugin);
            let stdout_path = output_dir.join("out");
            let stderr_path = output_dir.join("err");
            let cmdline_path = output_dir.join("cmdline");

            fs::create_dir_all(&output_dir)?;

            {
                let mut cmdline_file = File::create(cmdline_path)?;
                cmdline_file.write_all(&record.to_shell())?;
            }

            let stdout_file = File::create(stdout_path)?;
            let stderr_file = File::create(&stderr_path)?;

            let mut command: tokio::process::Command = command.into();
            command.kill_on_drop(true);
            command.stdin(Stdio::null());
            command.stdout(stdout_file);
            command.stderr(stderr_file);

            let task = async move {
                let mut child = command.spawn()?;
                let status = child.wait().await?;
                let log_name =
                    format!("COMPOSABLE_NODE {target_container_name} {package} {plugin}");
                save_status(&status, &output_dir, &log_name)?;

                eyre::Ok(())
            };

            eyre::Ok(task)
        })
        .collect();
    let load_node_tasks = load_node_tasks?;

    // Execute tasks simultaneously.
    let node_futures: FuturesUnordered<_> = node_tasks.into_iter().collect();
    let load_node_futures: FuturesUnordered<_> = load_node_tasks.into_iter().collect();

    join!(
        async move {
            node_futures
                .for_each(|result| async move {
                    if let Err(err) = result {
                        eprintln!("{err}");
                    }
                })
                .await;
        },
        async move {
            // Postpone the load node commands to wait for containers
            // to be ready.
            tokio::time::sleep(Duration::from_millis(5000)).await;

            load_node_futures
                .for_each(|result| async move {
                    if let Err(err) = result {
                        eprintln!("{err}");
                    }
                })
                .await;
        }
    );

    Ok(())
}

fn load_launch_dump(dump_file: &Path) -> eyre::Result<LaunchDump> {
    let reader = BufReader::new(File::open(dump_file)?);
    let launch_dump = serde_json::from_reader(reader)?;
    Ok(launch_dump)
}

fn load_and_transform_node_records<'a>(
    launch_dump: &'a LaunchDump,
    params_dir: &Path,
) -> eyre::Result<Vec<(&'a NodeRecord, NodeCommandLine)>> {
    let LaunchDump {
        node, file_data, ..
    } = launch_dump;

    let prepare: Result<Vec<_>, _> = node
        .par_iter()
        .map(|record| {
            let mut cmdline = NodeCommandLine::from_cmdline(&record.cmd)?;

            // Copy log_config_file to params dir.
            if let Some(src_path) = &cmdline.log_config_file {
                let Some(data) = file_data.get(src_path) else {
                    bail!(
                        "unable to find cached content for parameters file {}",
                        src_path.display()
                    );
                };
                cmdline.log_config_file = Some(copy_cached_data(src_path, params_dir, data)?);
            }

            // Copy params_file to params dir.
            cmdline.params_files = cmdline
                .params_files
                .iter()
                .map(|src_path| {
                    let Some(data) = file_data.get(src_path) else {
                        bail!(
                            "unable to find cached content for parameters file {}",
                            src_path.display()
                        );
                    };
                    let tgt_path = copy_cached_data(src_path, params_dir, data)?;
                    eyre::Ok(tgt_path)
                })
                .try_collect()?;

            eyre::Ok((record, cmdline))
        })
        .collect();

    prepare
}

fn copy_cached_data(src_path: &Path, tgt_dir: &Path, data: &str) -> eyre::Result<PathBuf> {
    let file_name = src_path.to_str().unwrap().replace(MAIN_SEPARATOR, "!");
    let tgt_path = tgt_dir.join(file_name);
    fs::write(&tgt_path, data)
        .wrap_err_with(|| format!("unable to create parameters file {}", tgt_path.display()))?;
    eyre::Ok(tgt_path)
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

fn save_status(status: &ExitStatus, output_dir: &Path, log_name: &str) -> eyre::Result<()> {
    let status_path = output_dir.join("status");

    // Save status code
    {
        let mut status_file = File::create(status_path)?;
        if let Some(code) = status.code() {
            write!(status_file, "{code}",)?;
        } else {
            write!(status_file, "[none]")?;
        }
    }

    // Print status to the terminal
    if status.success() {
        eprintln!("[{log_name}] finishes")
    } else {
        match status.code() {
            Some(code) => {
                eprintln!("[{log_name}] fails with code {code}.",);
                eprintln!("[{log_name}] Check {}", output_dir.display());
            }
            None => {
                eprintln!("[{log_name}] fails without exit code.",);
                eprintln!("[{log_name}] Check {}", output_dir.display());
            }
        }
    }

    eyre::Ok(())
}
