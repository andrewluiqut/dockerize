#! /bin/bash

# Basic Debugging Information
echo "> Configured for ROS $ROS_DISTRO"
echo "> List of Mounted Packages for Build in $SENSOR_WS"
ls $SENSOR_WS/src

# Build of Project Workspace
echo "> Building and Sourcing $SENSOR_WS"
source "$SENSOR_WS/devel/setup.bash"
cd $SENSOR_WS && catkin_make
source "$SENSOR_WS/devel/setup.bash"

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"