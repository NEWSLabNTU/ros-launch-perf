use crate::node_cmdline::NodeCommandLine;
use eyre::{bail, WrapErr};
use itertools::{chain, Itertools};
use rayon::prelude::*;
use serde::Deserialize;
use std::{
    borrow::Cow,
    collections::HashMap,
    fs::{self, File},
    io::BufReader,
    path::{Path, PathBuf, MAIN_SEPARATOR},
    process::Command,
};

pub type ParameterValue = String;

#[derive(Debug, Clone, Deserialize)]
pub struct LaunchDump {
    pub node: Vec<NodeRecord>,
    pub load_node: Vec<LoadNodeRecord>,
    pub container: Vec<ComposableNodeContainerRecord>,
    pub file_data: HashMap<PathBuf, String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ComposableNodeContainerRecord {
    pub namespace: String,
    pub name: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct NodeRecord {
    pub executable: String,
    pub package: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub exec_name: Option<String>,
    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,
    pub params_files: Vec<String>,
    #[serde(default)]
    pub remaps: Vec<(String, String)>,
    pub ros_args: Option<Vec<String>>,
    pub args: Option<Vec<String>>,
    pub cmd: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct LoadNodeRecord {
    pub package: String,
    pub plugin: String,
    pub target_container_name: String,
    pub node_name: String,
    pub namespace: String,
    pub log_level: Option<String>,

    #[serde(default)]
    pub remaps: Vec<(String, String)>,

    #[serde(default)]
    pub params: Vec<(String, ParameterValue)>,

    #[serde(default)]
    pub extra_args: HashMap<String, String>,
}

impl LoadNodeRecord {
    fn to_cmdline(&self, standalone: bool) -> Vec<String> {
        if standalone {
            self.to_cmdline_standalone()
        } else {
            self.to_cmdline_component()
        }
    }

    fn to_cmdline_standalone(&self) -> Vec<String> {
        let Self {
            package,
            plugin,
            namespace,
            log_level,
            remaps,
            params,
            extra_args,
            node_name,
            target_container_name: _,
        } = self;

        let command = [
            "ros2",
            "component",
            "standalone",
            package,
            plugin,
            "-n",
            node_name,
            "--node-namespace",
            namespace,
        ]
        .into_iter()
        .map(Cow::from);

        let remap_args = remaps
            .iter()
            .flat_map(|(src, tgt)| [Cow::from("-r"), format!("{src}:={tgt}").into()]);

        let param_args = params
            .iter()
            .flat_map(|(name, value)| [Cow::from("-p"), format!("{name}:={}", value).into()]);

        let extra_arg_args = extra_args
            .iter()
            .flat_map(|(name, value)| [Cow::from("-e"), format!("{name}:={}", value).into()]);

        let log_level_args = log_level
            .as_deref()
            .map(|level| ["--log-level", level])
            .into_iter()
            .flatten()
            .map(Cow::from);

        chain!(
            command,
            log_level_args,
            remap_args,
            param_args,
            extra_arg_args
        )
        .map(|arg| arg.to_string())
        .collect()
    }

    fn to_cmdline_component(&self) -> Vec<String> {
        let Self {
            package,
            plugin,
            namespace,
            log_level,
            remaps,
            params,
            extra_args,
            target_container_name,
            node_name,
        } = self;

        let command = [
            "ros2",
            "component",
            "load",
            target_container_name,
            package,
            plugin,
            "-n",
            node_name,
            "--node-namespace",
            namespace,
        ]
        .into_iter()
        .map(Cow::from);

        let remap_args = remaps
            .iter()
            .flat_map(|(src, tgt)| [Cow::from("-r"), format!("{src}:={tgt}").into()]);

        let param_args = params
            .iter()
            .flat_map(|(name, value)| [Cow::from("-p"), format!("{name}:={}", value).into()]);

        let extra_arg_args = extra_args
            .iter()
            .flat_map(|(name, value)| [Cow::from("-e"), format!("{name}:={}", value).into()]);

        let log_level_args = log_level
            .as_deref()
            .map(|level| ["--log-level", level])
            .into_iter()
            .flatten()
            .map(Cow::from);

        chain!(
            command,
            log_level_args,
            remap_args,
            param_args,
            extra_arg_args
        )
        .map(|arg| arg.to_string())
        .collect()
    }

    pub fn to_command(&self, standalone: bool) -> Command {
        let cmdline = self.to_cmdline(standalone);
        let (program, args) = cmdline.split_first().unwrap();
        let mut command = Command::new(program);
        command.args(args);
        command
    }

    pub fn to_shell(&self, standalone: bool) -> Vec<u8> {
        let cmdline = self.to_cmdline(standalone);
        Itertools::intersperse(
            cmdline.into_iter().map(|w| shell_quote::Sh::quote_vec(&w)),
            vec![b' '],
        )
        .flatten()
        .collect()
    }
}

pub fn load_launch_dump(dump_file: &Path) -> eyre::Result<LaunchDump> {
    let reader = BufReader::new(File::open(dump_file)?);
    let launch_dump = serde_json::from_reader(reader)?;
    Ok(launch_dump)
}

pub fn load_and_transform_node_records<'a>(
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
    let file_name = src_path.to_str().unwrap().replace(MAIN_SEPARATOR, "@");
    let tgt_path = tgt_dir.join(file_name);
    fs::write(&tgt_path, data)
        .wrap_err_with(|| format!("unable to create parameters file {}", tgt_path.display()))?;
    eyre::Ok(tgt_path)
}
