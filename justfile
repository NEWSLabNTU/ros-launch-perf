# ROS Launch Perf - 3-Stage Build System

set shell := ["bash", "-c"]

colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"
log_dir := "build_logs"

# Show available recipes
default:
    @just --list

# Install all dependencies
install-deps:
    git submodule update --init --recursive
    pip install git+https://github.com/colcon/colcon-cargo.git
    pip install git+https://github.com/colcon/colcon-ros-cargo.git
    cargo install cargo-ament-build
    . /opt/ros/humble/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Install ROS dependencies with rosdep
prepare:
    . /opt/ros/humble/setup.sh && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build entire project (all 3 stages)
build: build-ros2-rust build-interface build-packages

# Stage 1: Build ROS2 Rust base packages
build-ros2-rust:
    @echo "Stage 1: Building ROS2 Rust base packages... (log: {{log_dir}}/ros2_rust.log)"
    @mkdir -p {{log_dir}}
    . /opt/ros/humble/setup.sh && export RUST_LOG=info && colcon build {{colcon_flags}} --base-paths src/ros2_rust 2>&1 | tee {{log_dir}}/ros2_rust.log

# Stage 2: Build interface packages
build-interface:
    @echo "Stage 2: Building interface packages... (log: {{log_dir}}/interface.log)"
    @mkdir -p {{log_dir}}
    . install/setup.sh && export RUST_LOG=info && colcon build {{colcon_flags}} --base-paths src/interface 2>&1 | tee {{log_dir}}/interface.log

# Stage 3: Build application packages
build-packages: build-dump-launch build-play-launch build-tools

# Build dump_launch Python package
build-dump-launch:
    @echo "Stage 3: Building dump_launch... (log: {{log_dir}}/dump_launch.log)"
    @mkdir -p {{log_dir}}
    . install/setup.sh && export RUST_LOG=info && colcon build {{colcon_flags}} --base-paths src/dump_launch 2>&1 | tee {{log_dir}}/dump_launch.log

# Build play_launch Rust package
build-play-launch:
    @echo "Stage 3: Building play_launch... (log: {{log_dir}}/play_launch.log)"
    @mkdir -p {{log_dir}}
    . install/setup.sh && export RUST_LOG=info && colcon build {{colcon_flags}} --base-paths src/play_launch 2>&1 | tee {{log_dir}}/play_launch.log

# Build analysis tools and wrappers
build-tools:
    @echo "Stage 3: Building analysis tools and wrappers... (log: {{log_dir}}/tools.log)"
    @mkdir -p {{log_dir}}
    . install/setup.sh && colcon build {{colcon_flags}} --base-paths src/play_launch_wrapper src/play_launch_analyzer 2>&1 | tee {{log_dir}}/tools.log

# Apply CAP_SYS_PTRACE to I/O helper (requires sudo)
setcap-io-helper:
    #!/bin/bash
    if [ ! -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then
        echo "Error: play_launch_io_helper not found. Run 'just build-play-launch' first."
        exit 1
    fi
    sudo setcap cap_sys_ptrace+ep install/play_launch/lib/play_launch/play_launch_io_helper
    getcap install/play_launch/lib/play_launch/play_launch_io_helper
    echo "✓ I/O helper ready (reapply after rebuild)"

# Check I/O helper binary and capabilities
verify-io-helper:
    #!/bin/bash
    if [ -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then
        echo "✓ Binary exists"
        getcap install/play_launch/lib/play_launch/play_launch_io_helper || echo "⚠ CAP_SYS_PTRACE not set (run 'just setcap-io-helper')"
    else
        echo "✗ Binary not found (run 'just build-play-launch')"
    fi

# Clean all build artifacts
clean:
    rm -rf build install log .cargo {{log_dir}}

# Build Python wheels for packaging
build-wheels:
    @echo "Building Python wheels..."
    @mkdir -p wheels
    cd src/dump_launch && python3 -m pip wheel --no-deps -w ../../wheels .
    cd src/play_launch_analyzer && python3 -m pip wheel --no-deps -w ../../wheels .

# Build Debian package
build-deb: build-wheels
    ./scripts/build-deb.sh

# Run all tests
test:
    . install/setup.sh && colcon test --packages-select dump_launch play_launch && colcon test-result --all --verbose

# Run linters (clippy + ruff)
lint:
    . install/setup.sh && cargo clippy --all-targets --all-features -- -D warnings
    . install/setup.sh && cd src/dump_launch && python3 -m ruff check .

# Format code (Rust + Python)
format:
    cd src/play_launch && cargo +nightly fmt
    cd src/dump_launch && ruff format .
