# play-launch

ROS 2 launch inspection tool - record, replay, and monitor launch file executions.

## Features

- Record & replay ROS 2 launch files
- Per-node resource monitoring (CPU, memory, I/O, network, GPU)
- Interactive visualizations (Plotly)
- Component container support
- Auto-respawn nodes

## Install

**Requirements**: Ubuntu 22.04, ROS 2 Humble

```bash
# Install Pacstall
sudo bash -c "$(curl -fsSL https://pacstall.dev/q/install)"

# Install play-launch
pacstall -I play-launch
```

Build time: 10-15 min (x86_64), 20-30 min (arm64)

## Usage

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Record and replay
play_launch launch <package> <launch_file>

# With monitoring
play_launch launch <package> <launch_file> --enable-monitoring

# Plot results
play_launch plot

# I/O monitoring for containers (optional)
sudo /usr/share/play-launch/setup_io_helper.sh
```

## Components

- `play_launch` (6.5MB) - Main tool
- `dump_launch` - Launch recorder
- `play_launch_analyzer` - Log analyzer
- `play_launch_io_helper` (2.2MB) - I/O monitoring


## Documentation

- `/usr/share/doc/play-launch/INSTALLATION.md`
- `/usr/share/doc/play-launch/README.md`
- `/usr/share/play-launch/examples/`

## Uninstall

```bash
pacstall -R play-launch
```

## Support

- **Issues**: https://github.com/NEWSLabNTU/ros-launch-perf/issues
- **License**: BSD
- **Author**: Jerry Lin <jerry73204@gmail.com>
