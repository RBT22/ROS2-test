#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
# workaround for autocompletion adding to .bashrc
echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

cd /test_ws && colcon build
source /test_ws/install/local_setup.bash

exec "$@"