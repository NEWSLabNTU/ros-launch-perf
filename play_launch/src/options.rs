use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub struct Options {
    #[clap(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    #[clap(long, default_value = "record.json")]
    pub input_file: PathBuf,

    #[clap(long, default_value = "100")]
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

    /// Maximum time to wait for each container to be ready (seconds)
    #[clap(long, default_value = "30")]
    pub container_ready_timeout_secs: u64,

    /// Interval for polling container readiness (milliseconds)
    #[clap(long, default_value = "100")]
    pub container_poll_interval_ms: u64,

    /// Skip container readiness check and use fixed delay instead
    #[clap(long)]
    pub skip_container_check: bool,
}
