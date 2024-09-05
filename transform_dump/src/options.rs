use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub enum Options {
    GenerateScript(GenerateScript),
    Run(Run),
}

#[derive(Parser)]
pub struct GenerateScript {
    #[clap(long)]
    pub copy_params_dir: Option<PathBuf>,
    pub input_file: PathBuf,
}

#[derive(Parser)]
pub struct Run {
    #[clap(long)]
    pub copy_params_dir: Option<PathBuf>,
    pub input_file: PathBuf,
}
