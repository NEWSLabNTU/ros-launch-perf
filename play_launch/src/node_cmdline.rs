use crate::launch_dump::NodeRecord;
use eyre::{bail, Context};
use itertools::{chain, Itertools};
use serde::{Deserialize, Serialize};
use std::{
    borrow::{Borrow, Cow},
    collections::{HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    process::Command,
};

/// The command line information to execute a ROS node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NodeCommandLine {
    pub command: Vec<String>,
    pub user_args: Vec<String>,
    pub remaps: HashMap<String, String>,
    pub params: HashMap<String, String>,
    pub params_files: HashSet<PathBuf>,
    pub log_level: Option<String>,
    pub log_config_file: Option<PathBuf>,
    pub rosout_logs: Option<bool>,
    pub stdout_logs: Option<bool>,
    pub enclave: Option<String>,
}

impl NodeCommandLine {
    /// Construct from a ROS node record in the launch dump.
    pub fn from_node_record(record: &NodeRecord, params_files_dir: &Path) -> eyre::Result<Self> {
        let NodeRecord {
            executable,
            package,
            name,
            namespace,
            params,
            params_files: params_file_contents,
            remaps,
            ros_args: user_ros_args,
            args: user_nonros_args,
            exec_name: _,
            cmd: _,
        } = record;

        let Some(package) = package else {
            bail!(r#""package" is not set"#);
        };

        let command: Vec<_> = ["ros2", "run", package, executable]
            .into_iter()
            .map(|s| s.to_string())
            .collect();

        let user_args: Vec<_> = {
            let user_nonros_args = user_nonros_args.iter().flatten().map(|s| s.as_str());
            let user_ros_args = user_ros_args
                .iter()
                .flat_map(|args| chain!(["--ros-args"], args.iter().map(|s| s.as_str()), ["--"]));
            chain!(user_nonros_args, user_ros_args)
                .map(|s| s.to_string())
                .collect()
        };

        let remaps: HashMap<_, _> = {
            let name_remap = name
                .as_ref()
                .map(|name| ("__node".to_string(), name.to_string()));
            let namespace_remap = namespace
                .as_ref()
                .map(|namespace| ("__ns".to_string(), namespace.to_string()));
            let other_remaps = remaps
                .iter()
                .map(|(name, value)| (name.to_string(), value.to_string()));
            chain!(name_remap, namespace_remap, other_remaps).collect()
        };

        let params: HashMap<_, _> = params
            .iter()
            .map(|(name, value)| (name.to_string(), value.to_string()))
            .collect();

        let params_files: HashSet<_> = params_file_contents
            .iter()
            .enumerate()
            .map(|(idx, data)| {
                let file_name = format!("{idx}.yaml");
                let path = params_files_dir.join(file_name);
                fs::write(&path, data)
                    .wrap_err_with(|| format!("unable to write {}", path.display()))?;
                eyre::Ok(path)
            })
            .try_collect()?;

        Ok(Self {
            command,
            user_args,
            remaps,
            params,
            params_files,
            log_level: None,
            log_config_file: None,
            rosout_logs: None,
            stdout_logs: None,
            enclave: None,
        })
    }

    /// Construct from command line arguments.
    pub fn from_cmdline(cmdline: impl IntoIterator<Item = impl AsRef<str>>) -> eyre::Result<Self> {
        let (command, user_args, ros_args) = {
            let mut iter = cmdline.into_iter();
            let Some(command) = iter.next() else {
                bail!("the command line must not be empty");
            };

            let mut user_args = vec![];
            let mut ros_args = vec![];

            'arg_loop: loop {
                loop {
                    let Some(arg) = iter.next() else {
                        break 'arg_loop;
                    };

                    match arg.as_ref() {
                        "--ros-args" => break,
                        arg => {
                            user_args.push(arg.to_string());
                        }
                    }
                }

                loop {
                    let Some(arg) = iter.next() else {
                        break 'arg_loop;
                    };

                    match arg.as_ref() {
                        "--ros-args" => continue,
                        "--" => break,
                        _ => {
                            ros_args.push(arg);
                        }
                    }
                }
            }

            (command.as_ref().to_string(), user_args, ros_args)
        };

        let mut iter = ros_args.into_iter();
        let mut remaps = HashMap::new();
        let mut params = HashMap::new();
        let mut params_files = HashSet::new();
        let mut log_level = None;
        let mut log_config_file = None;
        let mut rosout_logs = None;
        let mut stdout_logs = None;
        let mut enclave = None;

        loop {
            let Some(arg) = iter.next() else { break };

            match arg.as_ref() {
                "-r" | "--remap" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -r/--remap");
                    };
                    let Some((name, value)) = arg2.as_ref().split_once(":=") else {
                        bail!("invalid assignment {}", arg2.as_ref())
                    };
                    remaps.insert(name.to_string(), value.to_string());
                }
                "-p" | "--param" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -p/--param");
                    };
                    let Some((name, value)) = arg2.as_ref().split_once(":=") else {
                        bail!("invalid assignment {}", arg2.as_ref())
                    };
                    params.insert(name.to_string(), value.to_string());
                }
                "--params-file" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -params-file");
                    };
                    params_files.insert(PathBuf::from(arg2.as_ref()));
                }
                "--log-level" => {
                    let Some(level) = iter.next() else {
                        bail!("expect an argument after --log-level");
                    };
                    log_level = Some(level.as_ref().to_string());
                }
                "--log-config-file" => {
                    let Some(path) = iter.next() else {
                        bail!("expect an argument after --log-config-file");
                    };
                    log_config_file = Some(PathBuf::from(path.as_ref()));
                }
                "--enable-rosout-logs" => {
                    rosout_logs = Some(true);
                }
                "--disable-rosout-logs" => {
                    rosout_logs = Some(false);
                }
                "--enable-stdout-logs" => {
                    stdout_logs = Some(true);
                }
                "--disable-stdout-logs" => {
                    stdout_logs = Some(false);
                }
                "-e" | "--enclave" => {
                    let Some(arg2) = iter.next() else {
                        bail!("expect an argument after -e/--enclave");
                    };
                    enclave = Some(arg2.as_ref().to_string())
                }
                arg => {
                    bail!("unexpected argument {arg} after --ros-arg");
                }
            }
        }

        Ok(Self {
            command: vec![command],
            user_args,
            remaps,
            params,
            params_files,
            log_level,
            log_config_file,
            rosout_logs,
            stdout_logs,
            enclave,
        })
    }

    /// Create command line arguments.
    pub fn to_cmdline(&self, long_args: bool) -> Vec<String> {
        let Self {
            command,
            user_args,
            remaps,
            params,
            params_files,
            log_level,
            log_config_file,
            rosout_logs,
            stdout_logs,
            enclave,
        } = self;

        let has_ros_args = !(remaps.is_empty()
            && params.is_empty()
            && params_files.is_empty()
            && log_level.is_none()
            && log_config_file.is_none()
            && rosout_logs.is_none()
            && stdout_logs.is_none()
            && enclave.is_none());

        let ros_args: Vec<_> = has_ros_args
            .then(|| {
                let param_switch = if long_args { "--param" } else { "-p" };
                let remap_switch = if long_args { "--remap" } else { "-r" };

                let log_level_args = log_level
                    .as_ref()
                    .map(|level| ["--log-level", level])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let log_config_file_args = log_config_file
                    .as_ref()
                    .map(|path| ["--log-config-file", path.to_str().unwrap()])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let rosout_logs_args = rosout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-rosout-logs"
                        } else {
                            "--disable-rosout-logs"
                        }
                    })
                    .map(Cow::from);
                let stdout_logs_args = stdout_logs
                    .map(|yes| {
                        if yes {
                            "--enable-stdout-logs"
                        } else {
                            "--disable-stdout-logs"
                        }
                    })
                    .map(Cow::from);
                let enclave_args = enclave
                    .as_ref()
                    .map(|value| ["--enclave", value])
                    .into_iter()
                    .flatten()
                    .map(Cow::from);
                let remap_args = remaps.iter().flat_map(move |(name, value)| {
                    [Cow::from(remap_switch), format!("{name}:={value}").into()]
                });
                let params_args = params.iter().flat_map(move |(name, value)| {
                    [Cow::from(param_switch), format!("{name}:={value}").into()]
                });
                let params_file_args = params_files
                    .iter()
                    .flat_map(|path| ["--params-file", path.to_str().unwrap()])
                    .map(Cow::from);

                chain!(
                    [Cow::from("--ros-args")],
                    log_level_args,
                    log_config_file_args,
                    rosout_logs_args,
                    stdout_logs_args,
                    enclave_args,
                    remap_args,
                    params_args,
                    params_file_args,
                )
            })
            .into_iter()
            .flatten()
            .collect();

        let words: Vec<_> = chain!(
            command.iter().map(|arg| arg.as_str()),
            user_args.iter().map(|arg| arg.as_str()),
            ros_args.iter().map(|arg| arg.borrow()),
        )
        .map(|arg| arg.to_string())
        .collect();

        words
    }

    /// Create a command object.
    pub fn to_command(&self, long_args: bool) -> Command {
        let cmdline = self.to_cmdline(long_args);
        let (program, args) = cmdline.split_first().unwrap();
        let mut command = Command::new(program);
        command.args(args);
        command
    }

    /// Generate the shell script.
    pub fn to_shell(&self, long_args: bool) -> Vec<u8> {
        Itertools::intersperse(
            self.to_cmdline(long_args)
                .into_iter()
                .map(|arg| shell_quote::Sh::quote_vec(&arg)),
            vec![b' '],
        )
        .flatten()
        .collect()
    }
}
