#! /bin/bash

echo "> Setting up ROS"
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "> Setting up catkin workspace"
source "$MOVEIT_WS/devel/setup.bash"

if [ "$ROS_MASTER" = true ]; then
    export ROS_MASTER_URI="http://localhost:11311"
    echo "> Setting up ROScore"
    /bin/bash -c "roscore || exit 0"
else
    if [[ -z "$ROS_MASTER_URI" ]]; then
        echo -e "\033[0;32mROS_MASTER_URL is not set\033[0m"
    fi
fi

# Run CMD from Dockerfile or the overriding command from docker compose yaml file
echo "> Running $@"
exec "$@"

