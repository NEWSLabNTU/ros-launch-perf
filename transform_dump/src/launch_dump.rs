use itertools::chain;
use serde::Deserialize;
use std::{borrow::Cow, collections::HashMap, path::PathBuf, process::Command};

pub type ParameterValue = String;

#[derive(Debug, Clone, Deserialize)]
pub struct LaunchDump {
    // pub process: Vec<ProcessRecord>,
    pub node: Vec<NodeRecord>,
    pub file_data: HashMap<PathBuf, String>,
    pub load_node: Vec<LoadNodeRecord>,
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

impl NodeRecord {
    pub fn to_shell(&self) -> String {
        let words = self.cmd.iter();
        let words: Vec<_> = words.map(|w| w.to_string()).collect();
        let words: Vec<_> = words.iter().map(|w| w.as_str()).collect();
        shellwords::join(&words)
    }
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
    fn to_cmdline_iter(&self) -> impl Iterator<Item = Cow<'_, str>> {
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
            .into_iter()
            .map(Cow::from);

        chain!(
            command,
            log_level_args,
            remap_args,
            param_args,
            extra_arg_args
        )
        .map(Cow::from)
    }

    pub fn to_command(&self) -> Command {
        let mut iter = self.to_cmdline_iter().map(|arg| arg.into_owned());
        let program = iter.next().unwrap();
        let args = iter;

        let mut command = Command::new(program);
        command.args(args);
        command
    }

    pub fn to_shell(&self) -> String {
        let words = self.to_cmdline_iter();
        let words: Vec<_> = words.map(|w| w.to_string()).collect();
        let words: Vec<_> = words.iter().map(|w| w.as_str()).collect();
        shellwords::join(&words)
    }
}
