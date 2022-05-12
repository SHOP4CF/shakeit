#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /ros_pkg_ws/install/setup.bash
ros2 launch shakeit_experiments run_sim_experiment.launch.py
exec "$@"