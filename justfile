set shell := ["zsh", "-cu"]

build:
    rye sync
    cargo build --release --all-targets

record launch_file *params:
    rye run dump_launch {{launch_file}} {{params}}

generate_script:
    mkdir -p params
    cargo run --release --bin play_launch -- --print-shell

play:
    mkdir -p params
    cargo run --release --bin play_launch --
    # cargo run --release --bin play_launch -- --standalone-composable-nodes
    # cargo run --release --bin play_launch -- --max-concurrent-load-node-spawn=100

profile:
    procpath record -p $(cat $(find log/node -name pid) | tr '\n' ',') -d profiling.sqlite -i 0.1

plot:
    mkdir plot

    procpath plot -d profiling.sqlite -q cpu -p $(cat $(find log/node -name pid | grep -v rviz) | tr '\n' ',' | head -c -1) -f plot/cpu-all.svg
    procpath plot -d profiling.sqlite -q rss -p $(cat $(find log/node -name pid | grep -v rviz) | tr '\n' ',' | head -c -1) -f plot/rss-all.svg

    cat $(find log/node -name pid | grep -v rviz) | \
    while read p; do; \
        echo procpath plot -d profiling.sqlite -q cpu -q rss -p $p -f plot/$p.svg; \
    done | \
    parallel -j0 --tty
