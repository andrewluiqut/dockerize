#! /bin/bash

# Basic Debugging Information
echo "> Configured for ROS $ROS_DISTRO"
echo "> List of Mounted Packages for Build in $PROJECT_WS"
ls $PROJECT_WS/src

# Build of Project Workspace
echo "> Building and Sourcing $PROJECT_WS"
source "$PROJECT_WS/install/setup.bash"
cd $PROJECT_WS && colcon build
source "$PROJECT_WS/install/setup.bash"

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"