#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ws/devel/setup.bash"
source "/catkin_ws/devel/setup.bash"
exec "$@"