SHELL := /bin/bash

.PHONY: build run plot clean

build:
	@echo "Building io_stress package..."
	@cd ../.. && colcon build --base-paths test/io_stress --symlink-install

run:
	@echo "Running I/O stress test with monitoring..."
	@. ../../install/setup.sh && play_launch launch io_stress io_stress.launch.xml --enable-monitoring

plot:
	@echo "Generating resource plots..."
	@play_launch plot

clean:
	@echo "Cleaning artifacts..."
	@rm -rf play_log record.json
