#!/usr/bin/bash

# This script sets up the environment and launches the pixel cm visualizer node.
# It first changes the current directory to the directory where the script is located.
# Then, it runs the cmd_build.bash script to build the necessary packages.
# After that, it sources the local_setup.bash script to set up the environment.
# Finally, it launches the pixel_cm_visualizer_launch.py file using ROS2.
set -e
cd "$(dirname "$0")"
./cmd_build.bash
source install/local_setup.bash
ros2 launch icar_bringup pixel_cm_visualizer_launch.py