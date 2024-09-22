use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub struct Options {
    #[clap(default_value = "log")]
    pub log_dir: PathBuf,

    #[clap(default_value = "record.json")]
    pub input_file: PathBuf,

    #[clap(default_value = "100")]
    pub delay_load_node_millis: u64,

    #[clap(default_value = "30000")]
    pub load_node_timeout_millis: u64,

    #[clap(default_value = "3")]
    pub load_node_attempts: usize,

    #[clap(default_value = "5")]
    pub max_concurrent_load_node_spawn: usize,

    #[clap(long)]
    pub print_shell: bool,
}
