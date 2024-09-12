mod launch_dump;
mod node_cmdline;
mod options;

use crate::{launch_dump::LoadNodeRecord, options::Options};
use clap::Parser;
use eyre::{bail, ensure, Context};
use futures::{join, stream::FuturesUnordered, StreamExt, TryStreamExt};
use itertools::Itertools;
use launch_dump::{ComposableNodeContainerRecord, LaunchDump, NodeRecord};
use node_cmdline::NodeCommandLine;
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs,
    fs::File,
    future::Future,
    io::{self, prelude::*, BufReader},
    path::{Path, PathBuf, MAIN_SEPARATOR},
    process::{ExitStatus, Stdio},
    time::Duration,
};
use tokio::runtime::Runtime;
use tracing::{error, info};

fn main() -> eyre::Result<()> {
    // install global collector configured based on RUST_LOG env var.
    tracing_subscriber::fmt::init();

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
    let node_cmdlines = load_and_transform_node_records(&launch_dump, &params_files_dir)?;
    let node_commands: Result<Vec<_>, _> = node_cmdlines
        .into_par_iter()
        .map(|(record, cmdline)| {
            let Some(exec_name) = &record.exec_name else {
                bail!(r#"expect the "exec_name" field but not found"#);
            };
            let Some(package) = &record.package else {
                bail!(r#"expect the "package" field but not found"#);
            };
            let output_dir = node_log_dir.join(package).join(exec_name);
            let exec = prepare_node_command(record, &cmdline, output_dir)?;

            eyre::Ok((record, cmdline, exec))
        })
        .collect();
    let node_commands = node_commands?;

    let (container_commands, pure_node_commands): (Vec<(String, _)>, Vec<_>) = node_commands
        .into_par_iter()
        .partition_map(|(record, cmdline, exec)| {
            use rayon::iter::Either;

            let NodeRecord {
                namespace, name, ..
            } = record;

            match (namespace, name) {
                (Some(namespace), Some(name)) => {
                    let container_key = format!("{namespace}/{name}");
                    if container_names.contains(&container_key) {
                        Either::Left((container_key, (record, cmdline, exec)))
                    } else {
                        Either::Right((record, cmdline, exec))
                    }
                }
                _ => Either::Right((record, cmdline, exec)),
            }
        });

    // Prepare LoadNode request execution contexts
    let load_node_records = &launch_dump.load_node;
    let load_node_commands: Result<Vec<_>, _> = load_node_records
        .par_iter()
        .map(|record| {
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
            let exec = prepare_load_node_command(record, output_dir)?;
            eyre::Ok((record, exec))
        })
        .collect();
    let load_node_commands = load_node_commands?;

    let (nice_load_node_tasks, orphan_load_node_commands): (Vec<_>, Vec<_>) = load_node_commands
        .into_par_iter()
        .partition_map(|(record, exec)| {
            use rayon::iter::Either;

            if container_names.contains(&record.target_container_name) {
                Either::Left((record, exec))
            } else {
                Either::Right((record, exec))
            }
        });

    // Report the number of entities
    info!("nodes:\t{}", pure_node_commands.len());
    info!("containers:\t{}", container_names.len());
    info!(
        "load node:\t{}",
        nice_load_node_tasks.len() + orphan_load_node_commands.len()
    );
    info!("orphan load node:\t{}", orphan_load_node_commands.len());

    // Build container groups
    let container_groups: HashMap<_, _> = {
        let mut container_groups: HashMap<_, _> = container_names
            .into_iter()
            .map(|container_name| {
                (
                    container_name,
                    ContainerGroup {
                        container_tasks: vec![],
                        load_node_tasks: vec![],
                    },
                )
            })
            .collect();

        for (container_name, (_record, _cmdline, exec)) in container_commands {
            container_groups
                .get_mut(&container_name)
                .unwrap()
                .container_tasks
                .push(exec);
        }

        for (record, exec) in nice_load_node_tasks {
            container_groups
                .get_mut(&record.target_container_name)
                .unwrap()
                .load_node_tasks
                .push(exec);
        }

        container_groups
    };

    // Execute tasks simultaneously.
    let container_group_futures: FuturesUnordered<_> = container_groups
        .into_iter()
        .map(|(_container_name, group)| {
            let ContainerGroup {
                container_tasks,
                load_node_tasks,
            } = group;

            let spawn_container_futures: FuturesUnordered<_> = container_tasks
                .into_iter()
                .map(|exec| async move {
                    let Execution {
                        log_name,
                        output_dir,
                        mut command,
                    } = exec;

                    let child = command.spawn().wrap_err_with(|| {
                        format!(
                            "{log_name} container is unable to start. Check {}",
                            output_dir.display()
                        )
                    })?;
                    let run_task =
                        async move { wait_for_node(&log_name, &output_dir, child).await };
                    eyre::Ok(run_task)
                })
                .collect();
            let run_load_node_futures: FuturesUnordered<_> = load_node_tasks
                .into_iter()
                .map(|exec| async move {
                    let Execution {
                        log_name,
                        output_dir,
                        mut command,
                    } = exec;
                    let child = command.spawn()?;
                    wait_for_load_node(&log_name, &output_dir, child).await?;
                    eyre::Ok(())
                })
                .collect();

            async move {
                let run_container_futures: FuturesUnordered<_> =
                    spawn_container_futures.try_collect().await?;
                tokio::time::sleep(load_node_delay).await;
                join!(
                    consume_futures(run_container_futures),
                    consume_futures(run_load_node_futures),
                );
                eyre::Ok(())
            }
        })
        .collect();

    let pure_node_futures: FuturesUnordered<_> = pure_node_commands
        .into_iter()
        .map(|(_, _, exec)| async move {
            let Execution {
                log_name,
                output_dir,
                mut command,
            } = exec;
            let child = command.spawn()?;
            wait_for_node(&log_name, &output_dir, child).await?;
            eyre::Ok(())
        })
        .collect();
    let orphan_load_node_futures: FuturesUnordered<_> = orphan_load_node_commands
        .into_iter()
        .map(|(_, exec)| async move {
            let Execution {
                log_name,
                output_dir,
                mut command,
            } = exec;
            let child = command.spawn()?;
            wait_for_load_node(&log_name, &output_dir, child).await?;
            eyre::Ok(())
        })
        .collect();

    join!(
        consume_futures(container_group_futures),
        consume_futures(pure_node_futures),
        consume_futures(orphan_load_node_futures),
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

fn save_node_status(status: &ExitStatus, output_dir: &Path, log_name: &str) -> eyre::Result<()> {
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
            write!(status_file, "{code}",)?;
        } else {
            write!(status_file, "[none]")?;
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

fn prepare_node_command<'a>(
    record: &'a NodeRecord,
    cmdline: &NodeCommandLine,
    output_dir: PathBuf,
) -> eyre::Result<Execution> {
    let Some(exec_name) = &record.exec_name else {
        bail!(r#"expect the "exec_name" field but not found"#);
    };
    let Some(package) = &record.package else {
        bail!(r#"expect the "package" field but not found"#);
    };

    // let output_dir = node_log_dir.join(package).join(exec_name);
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

    let log_name = format!("NODE {package} {exec_name}");
    // let task = async move {
    //     let mut child = command.spawn()?;
    //     let status = child.wait().await?;
    //     save_status(&status, &output_dir, &log_name)?;

    //     eyre::Ok(())
    // };

    eyre::Ok(Execution {
        log_name,
        output_dir,
        command,
    })
}

fn prepare_load_node_command(
    record: &LoadNodeRecord,
    output_dir: PathBuf,
) -> eyre::Result<Execution> {
    let command = record.to_command();
    let LoadNodeRecord {
        package,
        plugin,
        target_container_name,
        ..
    } = record;
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

    let log_name = format!("COMPOSABLE_NODE {target_container_name} {package} {plugin}");
    // let start_task = async move {
    //     let mut child = command.spawn()?;
    //     let status = child.wait().await?;
    //     save_status(&status, &output_dir, &log_name)?;
    //     eyre::Ok(())
    // };

    eyre::Ok(Execution {
        log_name,
        output_dir,
        command,
    })
}

async fn wait_for_node(
    log_name: &str,
    output_dir: &Path,
    mut child: tokio::process::Child,
) -> eyre::Result<()> {
    let status = child.wait().await?;
    save_node_status(&status, output_dir, log_name)?;
    eyre::Ok(())
}

async fn wait_for_load_node(
    log_name: &str,
    output_dir: &Path,
    mut child: tokio::process::Child,
) -> eyre::Result<()> {
    let status = child.wait().await?;
    save_load_node_status(&status, output_dir, log_name)?;
    eyre::Ok(())
}

async fn consume_futures<Fut>(futures: FuturesUnordered<Fut>)
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

struct ContainerGroup<CF, LF> {
    pub container_tasks: Vec<CF>,
    pub load_node_tasks: Vec<LF>,
}

struct Execution {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}
