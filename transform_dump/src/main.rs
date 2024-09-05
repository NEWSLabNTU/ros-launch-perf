mod launch_dump;

use clap::Parser;
use itertools::Itertools;
use launch_dump::LaunchDump;
use rayon::prelude::*;
use ros_cmdlines::CommandLine;
use std::{
    borrow::Borrow,
    fs,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

#[derive(Parser)]
struct Args {
    #[clap(long)]
    pub params_dir: PathBuf,
    pub input_file: PathBuf,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();
    let params_dir = args.params_dir.canonicalize()?;

    let copy_file = |src_path: &Path, data: &str| {
        let file_name = url_escape::encode_component(src_path.to_str().unwrap());
        let file_name: &str = file_name.borrow();
        let tgt_path = params_dir.join(file_name);
        fs::write(&tgt_path, data)?;
        eyre::Ok(tgt_path)
    };

    // Load the launch dump file.
    let LaunchDump { process, file_data } = {
        let reader = BufReader::new(File::open(&args.input_file)?);
        serde_json::from_reader(reader)?
    };

    let cmdlines: Result<Vec<_>, _> = process
        .par_iter()
        .map(|process| {
            let mut cmdline = CommandLine::from_cmdline(&process.cmdline)?;

            if let Some(src_path) = &cmdline.log_config_file {
                let Some(data) = file_data.get(src_path) else {
                    todo!();
                };
                cmdline.log_config_file = Some(copy_file(src_path, data)?);
            }

            cmdline.params_files = cmdline
                .params_files
                .iter()
                .map(|src_path| {
                    let Some(data) = file_data.get(src_path) else {
                        todo!();
                    };
                    let tgt_path = copy_file(src_path, data)?;
                    eyre::Ok(tgt_path)
                })
                .try_collect()?;

            let shell = cmdline.to_shell(false);
            eyre::Ok(shell)
        })
        .collect();
    let mut cmdlines = cmdlines?;
    cmdlines.par_sort_unstable();

    for cmdline in cmdlines {
        println!("{cmdline}");
    }

    Ok(())
}

// fn copy_file(src_path: &Path, tgt_dir: &Path) {
//     let file_name = url_escape::encode_component(src_path.to_str().unwrap());
//     let file_name: &str = file_name.borrow();
//     let tgt_path = tgt_dir.join(file_name);
//     fs::copy(src_path, &tgt_path).wrap_err_with(|| {
//         format!(
//             "unable to copy file {} to {}",
//             src_path.display(),
//             tgt_path.display()
//         )
//     })?;
//     eyre::Ok(tgt_path)
// }
