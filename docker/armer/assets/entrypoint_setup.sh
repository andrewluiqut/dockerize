#! /bin/bash

echo "> Setting up ROS"
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "> Setting up catkin workspace"
source "$ARMER_WS/devel/setup.bash"

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"

