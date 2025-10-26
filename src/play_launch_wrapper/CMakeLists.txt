cmake_minimum_required(VERSION 3.8)
project(play_launch_wrapper)

find_package(ament_cmake REQUIRED)

# Install the wrapper script
install(PROGRAMS
  scripts/play_launch
  DESTINATION lib/${PROJECT_NAME}
)

# Use ament_cmake's environment hook mechanism
ament_environment_hooks(hooks/play_launch_path.dsv.in)

ament_package()
