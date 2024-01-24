#! /bin/bash

echo "> Configured for ROS $ROS_DISTRO"
echo "> Sourcing workspace"
source "$MOVEIT_WS/install/setup.bash"

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"