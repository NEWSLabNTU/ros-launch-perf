COLCON_BUILD_FLAGS := --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
LOG_DIR := build_logs

.PHONY: default
default: help

.PHONY: help
help:
	@echo "ROS Launch Perf - 4-Stage Build System"
	@echo ""
	@echo "Build Commands:"
	@echo "  make build                  - Build entire project (all 4 stages)"
	@echo "  make build_ros2_rust        - Stage 1: Build ROS2 Rust base packages"
	@echo "  make build_interface        - Stage 2: Build ROS interface packages"
	@echo "  make build_dump_launch      - Stage 3: Build dump_launch Python package"
	@echo "  make build_play_launch      - Stage 4: Build play_launch Rust package"
	@echo ""
	@echo "Installation:"
	@echo "  make install                - Install binaries to ~/.cargo/bin"
	@echo "  make uninstall              - Remove installed binaries"
	@echo ""
	@echo "Development:"
	@echo "  make clean                  - Clean all build artifacts"
	@echo "  make test                   - Run all tests"
	@echo "  make format                 - Format code"
	@echo ""
	@echo "ROS Workspace Setup:"
	@echo "  make prepare                - Install ROS dependencies with rosdep"
	@echo ""

.PHONY: prepare
prepare:
	@echo "Installing ROS dependencies with rosdep..."
	@. /opt/ros/humble/setup.sh && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -r -y

.PHONY: build
build: build_ros2_rust build_interface build_dump_launch build_play_launch

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

.PHONY: install
install: build
	@echo "Installing binaries to ~/.cargo/bin..."
	@mkdir -p ~/.cargo/bin
	@cp install/play_launch/lib/play_launch/play_launch ~/.cargo/bin/
	@cp install/dump_launch/lib/dump_launch/dump_launch ~/.cargo/bin/
	@echo "Installation complete!"
	@echo "  dump_launch -> ~/.cargo/bin/dump_launch"
	@echo "  play_launch -> ~/.cargo/bin/play_launch"

.PHONY: uninstall
uninstall:
	@echo "Removing installed binaries..."
	@rm -f ~/.cargo/bin/dump_launch ~/.cargo/bin/play_launch
	@echo "Uninstallation complete!"

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

.PHONY: format
format:
	@echo "Formatting Rust code..."
	@cd src/play_launch && cargo +nightly fmt
	@echo "Formatting Python code..."
	@find src/dump_launch -name "*.py" -type f -not -path "*/test/*" | xargs -r python3 -m black --quiet 2>/dev/null || true
	@find src/dump_launch -name "*.py" -type f -not -path "*/test/*" | xargs -r python3 -m isort --quiet 2>/dev/null || true
