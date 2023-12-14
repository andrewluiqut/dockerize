#! /bin/bash

# echo ${ROS_DISTRO}
# echo ${ROS_MASTER}
# echo ${ROS_MASTER_URI}
# echo ${CATKIN_WS}

echo "> Setting up ROS"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# if ! [[ -z "$CATKIN_WS" ]]; then
#     echo "> Setting up catkin workspace"
#     source "${CATKIN_WS}/devel/setup.bash"
# fi

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
