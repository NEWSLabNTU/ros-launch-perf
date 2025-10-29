COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

.PHONY: default
default: help

.PHONY: help
help:
	@echo "ROS Launch Perf - 5-Stage Build System"
	@echo ""
	@echo "Setup Commands:"
	@echo "  make install-deps           - Install all dependencies (git submodules, colcon, cargo-ament, rosdep)"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build                  - Build entire project (all 5 stages)"
	@echo "  make build_ros2_rust        - Stage 1: Build ROS2 Rust base packages"
	@echo "  make build_interface        - Stage 2: Build ROS interface packages"
	@echo "  make build_dump_launch      - Stage 3: Build dump_launch Python package"
	@echo "  make build_play_launch      - Stage 4: Build play_launch Rust package"
	@echo "  make build_tools            - Stage 5: Build analysis tools and wrappers"
	@echo ""
	@echo "Development:"
	@echo "  make clean                  - Clean all build artifacts"
	@echo "  make test                   - Run all tests"
	@echo "  make lint                   - Run linters (clippy + ruff)"
	@echo "  make format                 - Format code"
	@echo ""
	@echo "ROS Workspace Setup:"
	@echo "  make prepare                - Install ROS dependencies with rosdep"
	@echo ""
	@echo "Usage:"
	@echo "  Source workspace:   . install/setup.bash"
	@echo "  Run dump_launch:    ros2 run dump_launch dump_launch <package> <launch_file> [args...]"
	@echo "  Run play_launch:    play_launch [options]"
	@echo "  Plot resources:     plot_play_launch [--log-dir <dir>]"
	@echo ""

.PHONY: install-deps
install-deps:
	@echo "Installing all dependencies..."
	@echo ""
	@echo "Step 1/4: Updating git submodules..."
	git submodule update --init --recursive
	@echo ""
	@echo "Step 2/4: Installing colcon extensions..."
	pip install git+https://github.com/colcon/colcon-cargo.git
	pip install git+https://github.com/colcon/colcon-ros-cargo.git
	@echo ""
	@echo "Step 3/4: Installing cargo-ament-build..."
	cargo install cargo-ament-build
	@echo ""
	@echo "Step 4/4: Installing ROS dependencies with rosdep..."
	@. /opt/ros/humble/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y
	@echo ""
	@echo "All dependencies installed successfully!"
	@echo "Next steps:"
	@echo "  1. Run 'make build' to build the project"
	@echo "  2. Source the workspace: . install/setup.bash"
	@echo "  3. Check help: play_launch --help"

.PHONY: prepare
prepare:
	@echo "Installing ROS dependencies with rosdep..."
	@. /opt/ros/humble/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build: build_ros2_rust build_interface build_dump_launch build_play_launch build_tools

.PHONY: build_ros2_rust
build_ros2_rust:
	@echo "Stage 1: Building ROS2 Rust base packages... (log: $(LOG_DIR)/ros2_rust.log)"
	@mkdir -p $(LOG_DIR)
	@. /opt/ros/humble/setup.sh && \
	export RUST_LOG=info && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/ros2_rust 2>&1 | tee $(LOG_DIR)/ros2_rust.log

.PHONY: build_interface
build_interface:
	@echo "Stage 2: Building interface packages... (log: $(LOG_DIR)/interface.log)"
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	export RUST_LOG=info && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/interface 2>&1 | tee $(LOG_DIR)/interface.log

.PHONY: build_dump_launch
build_dump_launch:
	@echo "Stage 3: Building dump_launch... (log: $(LOG_DIR)/dump_launch.log)"
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	export RUST_LOG=info && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/dump_launch 2>&1 | tee $(LOG_DIR)/dump_launch.log

.PHONY: build_play_launch
build_play_launch:
	@echo "Stage 4: Building play_launch... (log: $(LOG_DIR)/play_launch.log)"
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	export RUST_LOG=info && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/play_launch 2>&1 | tee $(LOG_DIR)/play_launch.log

.PHONY: build_tools
build_tools:
	@echo "Stage 5: Building analysis tools and wrappers... (log: $(LOG_DIR)/tools.log)"
	@mkdir -p $(LOG_DIR)
	@. install/setup.sh && \
	colcon build $(COLCON_BUILD_FLAGS) --base-paths src/play_launch_wrapper src/play_launch_analyzer 2>&1 | tee $(LOG_DIR)/tools.log

.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build install log .cargo $(LOG_DIR)
	@echo "Clean complete!"

.PHONY: test
test:
	@echo "Running tests..."
	@. install/setup.sh && \
	colcon test --packages-select dump_launch play_launch && \
	colcon test-result --all --verbose

.PHONY: lint
lint:
	@echo "Running Rust clippy..."
	@. install/setup.sh && \
	cargo clippy --all-targets --all-features -- -D warnings

	@echo "Running Python ruff checks..."
	@. install/setup.sh && \
	cd src/dump_launch && \
	python3 -m ruff check .

	@echo "Lint checks passed!"

.PHONY: format
format:
	@echo "Formatting Rust code..."
	@cd src/play_launch && cargo +nightly fmt
	@echo "Formatting Python code..."
	@cd src/dump_launch && ruff format .
