SHELL := /bin/bash
SCRIPT_DIR := $(shell pwd)
AUTOWARE_LINK := autoware
AUTOWARE_PATH := $(shell readlink -f $(AUTOWARE_LINK) 2>/dev/null)
AUTOWARE_SETUP := $(AUTOWARE_PATH)/install/setup.sh

MAP_PATH ?= $(HOME)/autoware_map/sample-map-planning

SERVICE_TIMEOUT ?= 300
LOAD_TIMEOUT ?= 60000

# Service name for systemd launch
SYSTEMD_SERVICE_NAME := autoware-planning-sim

.PHONY: help
help:
	@echo "Autoware Planning Simulator Test Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  start-sim          - Start Autoware planning simulator with play_launch"
	@echo ""
	@echo "Systemd Launch (ros2systemd):"
	@echo "  start-sim-systemd  - Start Autoware with ros2 systemd launch (as systemd user service)"
	@echo "  stop-sim-systemd   - Stop the Autoware systemd service"
	@echo "  status-sim-systemd - Show status of the Autoware systemd service"
	@echo "  logs-sim-systemd   - View recent logs from the Autoware systemd service"
	@echo ""
	@echo "Other:"
	@echo "  drive              - Run autonomous driving test"
	@echo "  start-sim-and-drive - Run complete test sequence (launch + autonomous test)"
	@echo "  plot               - Generate resource usage plots from latest play_log"
	@echo "  kill-orphans       - Kill orphan ROS nodes"
	@echo "  check-autoware     - Verify Autoware installation"
	@echo ""
	@echo "Variables:"
	@echo "  MAP_PATH=$(MAP_PATH)"
	@echo "  SERVICE_TIMEOUT=$(SERVICE_TIMEOUT)"
	@echo "  LOAD_TIMEOUT=$(LOAD_TIMEOUT)"

.PHONY: check-autoware
check-autoware:
	@if [ ! -L "$(AUTOWARE_LINK)" ]; then \
		echo "ERROR: 'autoware' symlink not found"; \
		echo "Create symlink: ln -s /path/to/autoware autoware"; \
		exit 1; \
	fi
	@if [ -z "$(AUTOWARE_PATH)" ] || [ ! -d "$(AUTOWARE_PATH)" ]; then \
		echo "ERROR: autoware symlink points to invalid path"; \
		exit 1; \
	fi
	@if [ ! -f "$(AUTOWARE_SETUP)" ]; then \
		echo "ERROR: install/setup.bash not found"; \
		exit 1; \
	fi
	@echo "Autoware installation: $(AUTOWARE_PATH)"

.PHONY: start-sim
start-sim: check-autoware
	@./scripts/start-sim.sh

.PHONY: start-sim-systemd
start-sim-systemd: check-autoware
	@echo "Starting Autoware as systemd user service..."
	@echo "Service name: $(SYSTEMD_SERVICE_NAME)"
	@echo "Map path: $(MAP_PATH)"
	@echo "CycloneDDS config: $(SCRIPT_DIR)/cyclonedds.xml"
	@echo "DISPLAY: $${DISPLAY:-<not set>}"
	@if [ ! -f "$(SCRIPT_DIR)/cyclonedds.xml" ]; then \
		echo "ERROR: CycloneDDS configuration file not found at $(SCRIPT_DIR)/cyclonedds.xml"; \
		exit 1; \
	fi
	@. $(AUTOWARE_SETUP) && \
	ros2 systemd launch \
		--name $(SYSTEMD_SERVICE_NAME) \
		--replace \
		--source $(AUTOWARE_SETUP) \
		--copy-env DISPLAY \
		--env CYCLONEDDS_URI=file://$(SCRIPT_DIR)/cyclonedds.xml \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(MAP_PATH)
	@echo ""
	@echo "Service started! Use the following commands to manage it:"
	@echo "  make status-sim-systemd  - Check service status"
	@echo "  make logs-sim-systemd    - View logs"
	@echo "  make stop-sim-systemd    - Stop the service"

.PHONY: stop-sim-systemd
stop-sim-systemd:
	@echo "Stopping Autoware systemd service..."
	@ros2 systemd stop $(SYSTEMD_SERVICE_NAME) || echo "Service may not be running"

.PHONY: status-sim-systemd
status-sim-systemd:
	@echo "Checking status of Autoware systemd service..."
	@ros2 systemd status $(SYSTEMD_SERVICE_NAME)

.PHONY: logs-sim-systemd
logs-sim-systemd:
	@echo "Showing recent logs from Autoware systemd service..."
	@echo "Press Ctrl+C to exit log viewer"
	@ros2 systemd logs $(SYSTEMD_SERVICE_NAME) -n 100 -f

.PHONY: drive
drive: check-autoware
	@echo "Running autonomous driving test..."
	@cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	python3 scripts/test_autonomous_drive.py

.PHONY: start-sim-and-drive
start-sim-and-drive: check-autoware
	@MAP_PATH=$(MAP_PATH) \
	SERVICE_TIMEOUT=$(SERVICE_TIMEOUT) \
	LOAD_TIMEOUT=$(LOAD_TIMEOUT) \
	./scripts/start-sim-and-drive.sh

.PHONY: plot
plot:
	@echo "Generating resource usage plots from latest play_log..."
	@play_launch plot

.PHONY: kill-orphans
kill-orphans:
	@echo "Killing orphan ROS nodes..."
	@./scripts/kill_orphan_nodes.sh
