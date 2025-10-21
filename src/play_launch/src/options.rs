use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub struct Options {
    #[clap(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    #[clap(long, default_value = "record.json")]
    pub input_file: PathBuf,

    #[clap(long, default_value = "2000")]
    pub delay_load_node_millis: u64,

    #[clap(long, default_value = "30000")]
    pub load_node_timeout_millis: u64,

    #[clap(long, default_value = "3")]
    pub load_node_attempts: usize,

    #[clap(long, default_value = "10")]
    pub max_concurrent_load_node_spawn: usize,

    #[clap(long)]
    pub standalone_composable_nodes: bool,

    #[clap(long)]
    pub load_orphan_composable_nodes: bool,

    #[clap(long)]
    pub print_shell: bool,

    /// Wait for container services to be available via ROS service discovery.
    /// If disabled, only waits for container processes to start (faster but less reliable).
    #[clap(long)]
    pub wait_for_service_ready: bool,

    /// Maximum time to wait for each container service (seconds).
    /// Set to 0 for unlimited wait time. Only used with --wait-for-service-ready.
    #[clap(long, default_value = "120")]
    pub service_ready_timeout_secs: u64,

    /// Interval for polling container service availability (milliseconds).
    /// Only used with --wait-for-service-ready.
    #[clap(long, default_value = "500")]
    pub service_poll_interval_ms: u64,

    /// Runtime configuration file (YAML).
    /// Contains monitoring settings, process control, and other runtime options.
    #[clap(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable resource monitoring for all nodes.
    /// This overrides the 'monitoring.enabled' setting in the config file.
    #[clap(long)]
    pub enable_monitoring: bool,

    /// Resource sampling interval in milliseconds (overrides config file).
    #[clap(long, value_name = "MS")]
    pub monitor_interval_ms: Option<u64>,
}
