# ROS2 Launch Inspection Tool

This project provides tools to record the execution of ROS 2 launch
and analyze the launch execution.

## Prerequisites

- Rust toolchain

  Visit [rustup.rs](https://rustup.rs/) and install `rustup`.

- rye

  Visit [rye.astral.sh](https://rye.astral.sh/) to install it. It is
  used to setup a Python virtual environment and to manage Python
  project.


- just

  Read the [book](https://just.systems/man/en/) and follow
  installation instructions. It enables us to use `justfile`, a modern
  version of `Makefile`.

## Usage

1. Record a launch execution.
  This example runs a Autoware planning simulation. Press Ctrl-C to
  terminate the record. A `dump.json` will be created.
  ```sh
  just record \
    /path_to_autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
  ```

2. Analyze the dump file.
  To generate the shell scripts to replay the launch on the terminal,
  ```sh
  just generate_script
  ```
  To replay the launch,
  ```sh
  just run_dump
  ```

## License

This software is distributed under MIT license. You can read the
[license file](LICENSE.txt).
