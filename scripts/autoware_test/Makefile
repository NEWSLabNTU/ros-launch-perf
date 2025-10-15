.PHONY: help run test clean kill-orphans check-autoware

SHELL := /bin/bash
SCRIPT_DIR := $(shell pwd)
AUTOWARE_LINK := autoware
AUTOWARE_PATH := $(shell readlink -f $(AUTOWARE_LINK) 2>/dev/null)
AUTOWARE_SETUP := $(AUTOWARE_PATH)/install/setup.bash

MAP_PATH ?= $(HOME)/autoware_map/sample-map-planning
VEHICLE_MODEL ?= sample_vehicle
SENSOR_MODEL ?= sample_sensor_kit

SERVICE_TIMEOUT ?= 300
LOAD_TIMEOUT ?= 60000

help:
	@echo "Autoware Test Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  run          - Start Autoware with play_launch"
	@echo "  test         - Run autonomous driving test"
	@echo "  full-test    - Run complete test sequence (launch + autonomous test)"
	@echo "  clean        - Kill orphan ROS nodes"
	@echo "  kill-orphans - Alias for clean"
	@echo "  check        - Verify Autoware installation"
	@echo ""
	@echo "Variables:"
	@echo "  MAP_PATH=$(MAP_PATH)"
	@echo "  VEHICLE_MODEL=$(VEHICLE_MODEL)"
	@echo "  SENSOR_MODEL=$(SENSOR_MODEL)"

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

run: check-autoware
	@echo "Starting Autoware with play_launch..."
	@cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	dump_launch \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(MAP_PATH) \
		vehicle_model:=$(VEHICLE_MODEL) \
		sensor_model:=$(SENSOR_MODEL) && \
	play_launch \
		--wait-for-service-ready \
		--service-ready-timeout-secs $(SERVICE_TIMEOUT) \
		--load-node-timeout-millis $(LOAD_TIMEOUT)

test: check-autoware
	@echo "Running autonomous driving test..."
	@cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	python3 test_autonomous_drive.py

full-test: check-autoware
	@echo "Starting full test sequence..."
	@echo "Step 1: Launching Autoware in background..."
	@cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	(dump_launch \
		autoware_launch planning_simulator.launch.xml \
		map_path:=$(MAP_PATH) \
		vehicle_model:=$(VEHICLE_MODEL) \
		sensor_model:=$(SENSOR_MODEL) && \
	play_launch \
		--wait-for-service-ready \
		--service-ready-timeout-secs $(SERVICE_TIMEOUT) \
		--load-node-timeout-millis $(LOAD_TIMEOUT)) & \
	LAUNCH_PID=$$!; \
	echo "play_launch started (PID: $$LAUNCH_PID)"; \
	echo "Step 2: Waiting 60s for system initialization..."; \
	sleep 60; \
	echo "Step 3: Running autonomous driving test..."; \
	cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	python3 test_autonomous_drive.py; \
	TEST_STATUS=$$?; \
	if [ $$TEST_STATUS -eq 0 ]; then \
		echo "Test completed successfully"; \
		echo "System running. Press Ctrl+C to stop."; \
		wait $$LAUNCH_PID; \
	else \
		echo "Test failed with exit code $$TEST_STATUS"; \
		kill $$LAUNCH_PID 2>/dev/null || true; \
		exit $$TEST_STATUS; \
	fi

kill-orphans:
	@echo "Killing orphan ROS nodes..."
	@bash ../kill_orphan_nodes.sh

clean: kill-orphans
