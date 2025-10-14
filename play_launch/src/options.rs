use clap::{Parser, ValueEnum};
use std::path::PathBuf;

/// Method for loading composable nodes into containers
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum LoadMethod {
    /// Use ROS service calls (faster, eliminates subprocess overhead)
    Service,
    /// Use `ros2 component load` subprocess (legacy method)
    Subprocess,
}

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

    /// Method for loading composable nodes into containers.
    /// 'service' uses direct ROS service calls (faster), 'subprocess' uses ros2 CLI (legacy).
    #[clap(long, value_enum, default_value = "subprocess")]
    pub load_method: LoadMethod,
}
