use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub enum Options {
    GenerateScript(GenerateScript),
    Play(Play),
}

#[derive(Parser)]
pub struct GenerateScript {
    #[clap(default_value = "log")]
    pub log_dir: PathBuf,
    #[clap(default_value = "record.json")]
    pub input_file: PathBuf,
}

#[derive(Parser)]
pub struct Play {
    #[clap(default_value = "log")]
    pub log_dir: PathBuf,
    #[clap(default_value = "record.json")]
    pub input_file: PathBuf,
    #[clap(default_value = "200")]
    pub delay_load_node_millis: u64,
}
