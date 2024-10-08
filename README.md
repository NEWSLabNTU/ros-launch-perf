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


- procpath

  Visit the [pypi site](https://pypi.org/project/Procpath/) to install
  this tool. This is used for resource usage profiling.

## Installation

Clone the project Git repository.

```sh
git clone https://github.com/NEWSLabNTU/ros-launch-perf.git
```

Compile the project and install the `dump_launch` and `play_launch`
commands on the system.

```sh
make install
```

You can uninstall these commands afterwards.

```sh
make uninstall
```


## Usage

### Record a Launch Execution

This example analyzes the launch file to run Autoware planning
simulation and generates `record.json` dump. This dump file records
all nodes and parameters to perform the simulation.

```sh
dump_launch \
  /path_to_autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

### Play the Launch

This command loads the `record.json` and perform the launch execution.

```sh
play_launch
```

## Profiling Resource Usage

Start the launch first by `just play`. Then, run this command to
profile per-process resource usage into `profiling.sqlite` database
file. Press Ctrl-C to terminate the profiling.

```sh
make profile
```

Afterwards, you can generate SVG timecharts using this command.

```sh
make plot
```

## License

This software is distributed under MIT license. You can read the
[license file](LICENSE.txt).
