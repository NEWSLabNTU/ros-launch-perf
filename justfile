build:
    rye sync
    cargo build --release --all-targets

record launch_file *params:
    rye run dump_launch {{launch_file}} {{params}}

generate_script:
    mkdir -p params
    cargo run --release --bin transform_dump -- generate-script

play_dump:
    mkdir -p params
    cargo run --release --bin transform_dump -- play
