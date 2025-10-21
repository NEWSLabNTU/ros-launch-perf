.PHONY: help start-sim drive start-sim-and-drive kill-orphans check-autoware

SHELL := /bin/bash
SCRIPT_DIR := $(shell pwd)
AUTOWARE_LINK := autoware
AUTOWARE_PATH := $(shell readlink -f $(AUTOWARE_LINK) 2>/dev/null)
AUTOWARE_SETUP := $(AUTOWARE_PATH)/install/setup.bash

MAP_PATH ?= $(HOME)/autoware_map/sample-map-planning

SERVICE_TIMEOUT ?= 300
LOAD_TIMEOUT ?= 60000

help:
	@echo "Autoware Planning Simulator Test Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  start-sim          - Start Autoware planning simulator with play_launch"
	@echo "  drive              - Run autonomous driving test"
	@echo "  start-sim-and-drive - Run complete test sequence (launch + autonomous test)"
	@echo "  kill-orphans       - Kill orphan ROS nodes"
	@echo "  check-autoware     - Verify Autoware installation"
	@echo ""
	@echo "Variables:"
	@echo "  MAP_PATH=$(MAP_PATH)"
	@echo "  SERVICE_TIMEOUT=$(SERVICE_TIMEOUT)"
	@echo "  LOAD_TIMEOUT=$(LOAD_TIMEOUT)"

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

start-sim: check-autoware
	@bash scripts/start-sim.sh

drive: check-autoware
	@echo "Running autonomous driving test..."
	@cd $(AUTOWARE_PATH)/install && source setup.bash 2>&1 >/dev/null && cd $(SCRIPT_DIR) && \
	python3 scripts/test_autonomous_drive.py

start-sim-and-drive: check-autoware
	@MAP_PATH=$(MAP_PATH) \
	SERVICE_TIMEOUT=$(SERVICE_TIMEOUT) \
	LOAD_TIMEOUT=$(LOAD_TIMEOUT) \
	./scripts/start-sim-and-drive.sh

kill-orphans:
	@echo "Killing orphan ROS nodes..."
	@./scripts/kill_orphan_nodes.sh
