use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub struct Options {
    #[clap(default_value = "log")]
    pub log_dir: PathBuf,

    #[clap(default_value = "record.json")]
    pub input_file: PathBuf,

    #[clap(default_value = "10")]
    pub delay_load_node_millis: u64,

    #[clap(long)]
    pub print_shell: bool,
}
