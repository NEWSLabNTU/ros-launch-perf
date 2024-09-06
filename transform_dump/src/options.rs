use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub enum Options {
    GenerateScript(GenerateScript),
    Play(Play),
}

#[derive(Parser)]
pub struct GenerateScript {
    #[clap(long)]
    pub copy_params_dir: Option<PathBuf>,
    pub input_file: PathBuf,
}

#[derive(Parser)]
pub struct Play {
    #[clap(long)]
    pub copy_params_dir: Option<PathBuf>,
    pub input_file: PathBuf,
}
