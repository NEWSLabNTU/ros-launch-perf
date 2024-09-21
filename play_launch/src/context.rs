use crate::{
    launch_dump::{load_and_transform_node_records, LaunchDump, LoadNodeRecord, NodeRecord},
    node_cmdline::NodeCommandLine,
};
use eyre::{bail, ensure};
use rayon::prelude::*;
use std::{
    collections::HashSet,
    fs,
    fs::File,
    io::prelude::*,
    path::{Path, PathBuf},
    process::Stdio,
};

pub struct NodeContextSet<'a> {
    pub container_contexts: Vec<NodeContainerContext<'a>>,
    pub pure_node_contexts: Vec<NodeContext<'a>>,
}

pub struct LoadNodeContextSet<'a> {
    pub nice_load_node_contexts: Vec<LoadNodeContext<'a>>,
    pub orphan_load_node_contexts: Vec<LoadNodeContext<'a>>,
}

pub struct LoadNodeContext<'a> {
    pub record: &'a LoadNodeRecord,
    pub exec: ExecutionContext,
}

pub struct NodeContext<'a> {
    pub record: &'a NodeRecord,
    pub cmdline: NodeCommandLine,
    pub exec: ExecutionContext,
}

pub struct NodeContainerContext<'a> {
    pub node_container_name: String,
    pub node_context: NodeContext<'a>,
}

pub struct ExecutionContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}

pub fn prepare_node_contexts<'a>(
    launch_dump: &'a LaunchDump,
    params_files_dir: &Path,
    node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<NodeContextSet<'a>> {
    let node_cmdlines = load_and_transform_node_records(launch_dump, params_files_dir)?;
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

    let (container_contexts, pure_node_contexts): (Vec<_>, Vec<_>) = node_commands
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
                        Either::Left(NodeContainerContext {
                            node_container_name: container_key,
                            node_context: NodeContext {
                                record,
                                cmdline,
                                exec,
                            },
                        })
                    } else {
                        Either::Right(NodeContext {
                            record,
                            cmdline,
                            exec,
                        })
                    }
                }
                _ => Either::Right(NodeContext {
                    record,
                    cmdline,
                    exec,
                }),
            }
        });

    Ok(NodeContextSet {
        container_contexts,
        pure_node_contexts,
    })
}

pub fn prepare_load_node_contexts<'a>(
    launch_dump: &'a LaunchDump,
    load_node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<LoadNodeContextSet<'a>> {
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
            eyre::Ok(LoadNodeContext { record, exec })
        })
        .collect();
    let load_node_commands = load_node_commands?;

    let (nice_load_node_contexts, orphan_load_node_contexts): (Vec<_>, Vec<_>) =
        load_node_commands.into_par_iter().partition_map(|context| {
            use rayon::iter::Either;

            if container_names.contains(&context.record.target_container_name) {
                Either::Left(context)
            } else {
                Either::Right(context)
            }
        });

    Ok(LoadNodeContextSet {
        nice_load_node_contexts,
        orphan_load_node_contexts,
    })
}

fn prepare_node_command(
    record: &NodeRecord,
    cmdline: &NodeCommandLine,
    output_dir: PathBuf,
) -> eyre::Result<ExecutionContext> {
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
    eyre::Ok(ExecutionContext {
        log_name,
        output_dir,
        command,
    })
}

fn prepare_load_node_command(
    record: &LoadNodeRecord,
    output_dir: PathBuf,
) -> eyre::Result<ExecutionContext> {
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

    eyre::Ok(ExecutionContext {
        log_name,
        output_dir,
        command,
    })
}
