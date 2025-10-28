use clap::{Args, Parser, Subcommand};
use std::path::PathBuf;

/// Record and replay ROS 2 launches with inspection capabilities
#[derive(Parser)]
#[command(name = "play_launch")]
#[command(version)]
#[command(about = "Record and replay ROS 2 launches with inspection capabilities")]
#[command(after_help = "Examples:\n  \
    play_launch launch demo_nodes_cpp talker_listener.launch.py\n  \
    play_launch run demo_nodes_cpp talker\n  \
    play_launch dump launch autoware_launch planning_simulator.launch.xml --output autoware.json\n  \
    play_launch replay --input-file record.json")]
#[command(arg_required_else_help = true)]
pub struct Options {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Launch a ROS 2 launch file (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch launch demo_nodes_cpp talker_listener.launch.py\n  \
        play_launch launch /path/to/launch.py use_sim_time:=true")]
    Launch(LaunchArgs),

    /// Run a single ROS 2 node (dump + replay)
    #[command(after_help = "Examples:\n  \
        play_launch run demo_nodes_cpp talker\n  \
        play_launch run demo_nodes_cpp talker --ros-args -p topic:=chatter")]
    Run(RunArgs),

    /// Dump launch execution without replaying
    Dump(DumpArgs),

    /// Replay from existing record.json
    #[command(after_help = "Examples:\n  \
        play_launch replay\n  \
        play_launch replay --input-file autoware.json --enable-monitoring")]
    Replay(ReplayArgs),
}

/// Arguments for launching a launch file
#[derive(Args)]
pub struct LaunchArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format
    #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
    pub launch_arguments: Vec<String>,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for running a single node
#[derive(Args)]
pub struct RunArgs {
    /// Package name
    pub package: String,

    /// Executable name
    pub executable: String,

    /// Node arguments
    #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
    pub args: Vec<String>,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for dump command
#[derive(Args)]
pub struct DumpArgs {
    #[command(subcommand)]
    pub subcommand: DumpSubcommand,

    /// Output file for the dump
    #[arg(long, short = 'o', default_value = "record.json")]
    pub output: PathBuf,

    /// Enable debug output during dump
    #[arg(long)]
    pub debug: bool,
}

#[derive(Subcommand)]
pub enum DumpSubcommand {
    /// Dump a launch file execution
    Launch(LaunchArgs),
    /// Dump a single node execution
    Run(RunArgs),
}

/// Arguments for replay command
#[derive(Args, Default)]
pub struct ReplayArgs {
    /// Input record file to replay
    #[arg(long, default_value = "record.json")]
    pub input_file: PathBuf,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Common options shared across all commands
#[derive(Args, Default, Clone)]
pub struct CommonOptions {
    /// Log directory for execution outputs
    #[arg(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    /// Delay before loading composable nodes (milliseconds)
    #[arg(long, default_value = "2000")]
    pub delay_load_node_millis: u64,

    /// Timeout for loading each composable node (milliseconds)
    #[arg(long, default_value = "30000")]
    pub load_node_timeout_millis: u64,

    /// Maximum retry attempts for loading composable nodes
    #[arg(long, default_value = "3")]
    pub load_node_attempts: usize,

    /// Maximum concurrent composable node loading operations
    #[arg(long, default_value = "10")]
    pub max_concurrent_load_node_spawn: usize,

    /// Run composable nodes in standalone mode instead of loading into containers
    #[arg(long)]
    pub standalone_composable_nodes: bool,

    /// Load composable nodes that have no matching container
    #[arg(long)]
    pub load_orphan_composable_nodes: bool,

    /// Generate shell script instead of executing
    #[arg(long)]
    pub print_shell: bool,

    /// Wait for container services to be available via ROS service discovery.
    /// If disabled, only waits for container processes to start (faster but less reliable).
    #[arg(long)]
    pub wait_for_service_ready: bool,

    /// Maximum time to wait for each container service (seconds).
    /// Set to 0 for unlimited wait time. Only used with --wait-for-service-ready.
    #[arg(long, default_value = "120")]
    pub service_ready_timeout_secs: u64,

    /// Interval for polling container service availability (milliseconds).
    /// Only used with --wait-for-service-ready.
    #[arg(long, default_value = "500")]
    pub service_poll_interval_ms: u64,

    /// Runtime configuration file (YAML).
    /// Contains monitoring settings, process control, and other runtime options.
    #[arg(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable resource monitoring for all nodes.
    /// This overrides the 'monitoring.enabled' setting in the config file.
    #[arg(long)]
    pub enable_monitoring: bool,

    /// Resource sampling interval in milliseconds (overrides config file).
    #[arg(long, value_name = "MS")]
    pub monitor_interval_ms: Option<u64>,
}
