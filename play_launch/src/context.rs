use crate::{
    launch_dump::{LaunchDump, LoadNodeRecord, NodeRecord},
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
use tokio::process::Command;

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

impl LoadNodeContext {
    pub fn to_load_node_command(&self, round: usize) -> eyre::Result<Command> {
        let LoadNodeContext {
            output_dir, record, ..
        } = self;

        let command = record.to_command(false);
        let stdout_path = output_dir.join(format!("out.{round}"));
        let stderr_path = output_dir.join(format!("err.{round}"));
        let cmdline_path = output_dir.join(format!("cmdline.{round}"));

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&record.to_shell(false))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: Command = command.into();
        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }

    pub fn to_standalone_node_command(&self) -> eyre::Result<Command> {
        let LoadNodeContext {
            output_dir, record, ..
        } = self;

        let command = record.to_command(true);
        let stdout_path = output_dir.join("out");
        let stderr_path = output_dir.join("err");
        let cmdline_path = output_dir.join("cmdline");

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&record.to_shell(true))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: Command = command.into();
        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }
}

pub struct NodeContext {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
}

impl NodeContext {
    pub fn to_exec_context(&self) -> eyre::Result<ExecutionContext> {
        let NodeContext {
            record,
            cmdline,
            output_dir,
        } = self;

        let Some(exec_name) = &record.exec_name else {
            bail!(r#"expect the "exec_name" field but not found"#);
        };
        let Some(package) = &record.package else {
            bail!(r#"expect the "package" field but not found"#);
        };

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
        // let mut command: tokio::process::Command = record.to_command(false).into();
        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        let log_name = format!("NODE {package} {exec_name}");
        Ok(ExecutionContext {
            log_name,
            output_dir: output_dir.to_path_buf(),
            command,
        })
    }
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
    node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<NodeContextSet> {
    let node_contexts: Result<Vec<_>, _> = launch_dump
        .node
        .par_iter()
        .map(|record| {
            let Some(exec_name) = &record.exec_name else {
                bail!(r#"expect the "exec_name" field but not found"#);
            };
            let Some(package) = &record.package else {
                bail!(r#"expect the "package" field but not found"#);
            };
            let output_dir = node_log_dir.join(package).join(exec_name);
            let params_files_dir = output_dir.join("params_files");

            fs::create_dir_all(&params_files_dir)?;
            let cmdline = NodeCommandLine::from_node_record(record, &params_files_dir)?;

            eyre::Ok(NodeContext {
                record: record.clone(),
                cmdline,
                output_dir,
            })
        })
        .collect();
    let node_contexts = node_contexts?;

    let (container_contexts, pure_node_contexts): (Vec<_>, Vec<_>) =
        node_contexts.into_par_iter().partition_map(|context| {
            use rayon::iter::Either;

            let NodeContext {
                record: NodeRecord {
                    name, namespace, ..
                },
                ..
            } = &context;

            match (namespace, name) {
                (Some(namespace), Some(name)) => {
                    let container_key = format!("{namespace}/{name}");
                    if container_names.contains(&container_key) {
                        Either::Left(NodeContainerContext {
                            node_container_name: container_key,
                            node_context: context,
                        })
                    } else {
                        Either::Right(context)
                    }
                }
                _ => Either::Right(context),
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
