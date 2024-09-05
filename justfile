build:
    rye sync
    cargo build --release --all-targets

record launch_file +params:
    rye run dump_launch -o dump.json {{launch_file}} {{params}}

generate_script:
    mkdir -p params
    ./target/release/transform_dump generate-script --copy-params-dir params/ dump.json

run_dump:
    mkdir -p params
    ./target/release/transform_dump run --copy-params-dir params/ dump.json
