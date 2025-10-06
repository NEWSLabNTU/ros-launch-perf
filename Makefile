SHELL := bash

.DEFAULT_GOAL := help

.PHONY: help
help:  ## Show this help message
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

.PHONY: build
build:  ## Build both dump_launch and play_launch packages
	cd dump_launch && uv sync && uv build
	cd play_launch && cargo build --release

.PHONY: clean
clean:  ## Clean build artifacts
	rm -rf dump_launch/dist
	cd play_launch && cargo clean

.PHONY: install
install: build  ## Install both packages
	uv pip install -U dump_launch/dist/dump_launch-0.1.0-py3-none-any.whl
	cargo install --path play_launch

.PHONY: debian
debian:  ## Build Debian package
	cargo deb

.PHONY: uninstall
uninstall:  ## Uninstall both packages
	pip uninstall -y dump_launch
	cargo uninstall play_launch

.PHONY: profile
profile:  ## Profile resource usage (requires procpath)
	procpath record -p $(cat $(find log/node -name pid) | tr '\n' ',') -d profiling.sqlite -i 0.1

.PHONY: plot
plot:  ## Generate profiling plots
	mkdir plot

	procpath plot -d profiling.sqlite -q cpu -p $(cat $(find log/node -name pid | grep -v rviz) | tr '\n' ',' | head -c -1) -f plot/cpu-all.svg
	procpath plot -d profiling.sqlite -q rss -p $(cat $(find log/node -name pid | grep -v rviz) | tr '\n' ',' | head -c -1) -f plot/rss-all.svg

	cat $(find log/node -name pid | grep -v rviz) | \
	while read p; do; \
	    echo procpath plot -d profiling.sqlite -q cpu -q rss -p $p -f plot/$p.svg; \
	done | \
	parallel -j0 --tty
