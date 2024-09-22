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

pub struct NodeContextSet {
    pub container_contexts: Vec<NodeContainerContext>,
    pub noncontainer_node_contexts: Vec<NodeContext>,
}

pub struct LoadNodeContextSet {
    pub load_node_contexts: Vec<LoadNodeContext>,
}

pub struct LoadNodeContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub record: LoadNodeRecord,
}

pub struct NodeContext {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub exec: ExecutionContext,
}

pub struct NodeContainerContext {
    pub node_container_name: String,
    pub node_context: NodeContext,
}

pub struct ExecutionContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}

pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    params_files_dir: &Path,
    node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<NodeContextSet> {
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
                                record: record.clone(),
                                cmdline,
                                exec,
                            },
                        })
                    } else {
                        Either::Right(NodeContext {
                            record: record.clone(),
                            cmdline,
                            exec,
                        })
                    }
                }
                _ => Either::Right(NodeContext {
                    record: record.clone(),
                    cmdline,
                    exec,
                }),
            }
        });

    Ok(NodeContextSet {
        container_contexts,
        noncontainer_node_contexts: pure_node_contexts,
    })
}

pub fn prepare_load_node_contexts(
    launch_dump: &LaunchDump,
    load_node_log_dir: &Path,
) -> eyre::Result<LoadNodeContextSet> {
    let load_node_records = &launch_dump.load_node;
    let load_node_contexts: Result<Vec<_>, _> = load_node_records
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
            let log_name = format!("COMPOSABLE_NODE {target_container_name} {package} {plugin}");
            eyre::Ok(LoadNodeContext {
                record: record.clone(),
                log_name,
                output_dir,
            })
        })
        .collect();
    let load_node_contexts = load_node_contexts?;

    Ok(LoadNodeContextSet { load_node_contexts })
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
