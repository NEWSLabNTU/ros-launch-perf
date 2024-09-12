# ROS2 Launch Inspection Tool

This project provides tools to record the execution of ROS 2
launch. Then, analyze and replay the launch execution.

[![Watch the demo](demo.png)](demo.webm)

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

### Record a launch execution.

This example runs a Autoware planning simulation. Press Ctrl-C to
terminate the record. A `record.json` will be created.

```sh
just record \
  /path_to_autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

### Analyze the dump file.

To replay the launch,

```sh
just play
```

To generate the shell scripts to replay the launch on the terminal,

```sh
just generate_script
```

## Known Issues

### Racing Among Composable Node Containers and LoadNode Requests

The ROS2 launch provides the composable node container feature, which
starts a special node as a container, and the container process
receives composable node requests afterwards. In cases that tens of
node processes are spawned, the container may not be ready before
LoadNode requests are sent due to heavy system load.

### Incorrect Parameter Type

The node parameters are recorded and are stored in the JSON dump. It
is observed that the `just play_dump` may not correctly interpret the
parameter type.

## License

This software is distributed under MIT license. You can read the
[license file](LICENSE.txt).
