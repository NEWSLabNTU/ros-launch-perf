.PHONY: build install uninstall dump play generate_script profile plot clean
SHELL := bash

build:
	rye sync
	rye build --all
	cargo build --release --all-targets

clean:
	rm -rf dist
	cargo clean

install: build
	pip install -U dist/dump_launch-0.1.0-py3-none-any.whl
	cargo install --path play_launch

debian:
	cargo deb

uninstall:
	pip uninstall -y dump_launch
	cargo uninstall play_launch

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
