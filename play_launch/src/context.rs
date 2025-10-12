use crate::{
    launch_dump::{ComposableNodeRecord, LaunchDump, NodeRecord},
    node_cmdline::NodeCommandLine,
};
use eyre::bail;
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

/// ROS node contexts classified into disjoint sets.
pub struct NodeContextClasses {
    pub container_contexts: Vec<NodeContainerContext>,
    pub non_container_node_contexts: Vec<NodeContext>,
}

/// A set of composable node contexts belonging to the same node
/// container.
pub struct ComposableNodeContextSet {
    pub load_node_contexts: Vec<ComposableNodeContext>,
}

/// The context contains all essential data to load a ROS composable
/// node into a node container.
pub struct ComposableNodeContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub record: ComposableNodeRecord,
}

impl ComposableNodeContext {
    pub fn to_load_node_command(&self, round: usize) -> eyre::Result<Command> {
        let ComposableNodeContext {
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

        // Create a new process group to ensure all child processes are killed together
        #[cfg(unix)]
        unsafe {
            command.pre_exec(|| {
                libc::setpgid(0, 0);
                Ok(())
            });
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }

    pub fn to_standalone_node_command(&self) -> eyre::Result<Command> {
        let ComposableNodeContext {
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

        // Create a new process group to ensure all child processes are killed together
        #[cfg(unix)]
        unsafe {
            command.pre_exec(|| {
                libc::setpgid(0, 0);
                Ok(())
            });
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }
}

/// The context contains all essential data to execute a ROS node.
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

        let stdout_path = output_dir.join("out");
        let stderr_path = output_dir.join("err");
        let cmdline_path = output_dir.join("cmdline");

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&cmdline.to_shell(false))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: tokio::process::Command = cmdline.to_command(false).into();
        // let mut command: tokio::process::Command = record.to_command(false).into();

        // Create a new process group to ensure all child processes are killed together
        #[cfg(unix)]
        unsafe {
            command.pre_exec(|| {
                // Create a new process group
                libc::setpgid(0, 0);
                Ok(())
            });
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        let log_name = format!("NODE '{package}/{exec_name}'");
        Ok(ExecutionContext {
            log_name,
            output_dir: output_dir.to_path_buf(),
            command,
        })
    }
}

/// The context contains all essential data to execute a node
/// container.
pub struct NodeContainerContext {
    pub node_container_name: String,
    pub node_context: NodeContext,
}

/// Essential data for a process execution.
pub struct ExecutionContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}

/// Load and classify node records in the dump by its kind.
pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<NodeContextClasses> {
    // Prepare node contexts from node records in the dump.
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

            // Create the dir for the node
            let output_dir = node_log_dir.join(package).join(exec_name);
            let params_files_dir = output_dir.join("params_files");
            fs::create_dir_all(&params_files_dir)?;

            // Build the command line.
            let cmdline = NodeCommandLine::from_node_record(record, &params_files_dir)?;

            eyre::Ok(NodeContext {
                record: record.clone(),
                cmdline,
                output_dir,
            })
        })
        .collect();
    let node_contexts = node_contexts?;

    // Classify the node contexts by checking if it's a node container
    // or not.
    let (container_contexts, non_container_node_contexts): (Vec<_>, Vec<_>) =
        node_contexts.into_par_iter().partition_map(|context| {
            use rayon::iter::Either;

            let NodeContext {
                record: NodeRecord {
                    name, namespace, ..
                },
                ..
            } = &context;

            // Check if the package/node_name is a known node
            // container name.
            let container_name = {
                let (Some(namespace), Some(name)) = (namespace, name) else {
                    return Either::Right(context);
                };

                // Handle namespace ending with '/' to avoid double slashes
                let container_name = if namespace.ends_with('/') {
                    format!("{namespace}{name}")
                } else {
                    format!("{namespace}/{name}")
                };
                if !container_names.contains(&container_name) {
                    return Either::Right(context);
                };
                container_name
            };
            let container_context = NodeContainerContext {
                node_container_name: container_name,
                node_context: context,
            };
            Either::Left(container_context)
        });

    Ok(NodeContextClasses {
        container_contexts,
        non_container_node_contexts,
    })
}

/// Load composable node records in the dump.
pub fn prepare_composable_node_contexts(
    launch_dump: &LaunchDump,
    load_node_log_dir: &Path,
) -> eyre::Result<ComposableNodeContextSet> {
    let load_node_records = &launch_dump.load_node;
    let load_node_contexts: Result<Vec<_>, _> = load_node_records
        .par_iter()
        .map(|record| {
            let ComposableNodeRecord {
                package,
                plugin,
                target_container_name,
                ..
            } = record;
            let output_dir = load_node_log_dir
                .join(target_container_name.replace("/", "@"))
                .join(package)
                .join(plugin);
            let log_name = format!("COMPOSABLE_NODE '{package}/{plugin}'");
            eyre::Ok(ComposableNodeContext {
                record: record.clone(),
                log_name,
                output_dir,
            })
        })
        .collect();
    let load_node_contexts = load_node_contexts?;

    Ok(ComposableNodeContextSet { load_node_contexts })
}
