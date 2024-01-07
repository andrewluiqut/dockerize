#! /bin/bash

#------------------ Status Display Setup ---------------------------
# ANSI escape sequence for select graphic rendition
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# Printed message to user terminal on open
echo "##########################################################################"

figlet -w 74 -c "$(hostname)"

echo -e "> Robot Name:\t\t\t$(hostname)"
# Status check of robot operating system (roscore) daemon
ROS_STATUS="${RED}DEAD${NC}"
# Check if rostopic list command results in an error
# Send output (stdout and stderr) to /dev/null (to mute to user)

curl -s $ROS_MASTER_URI > /dev/null

if [ "$?" == "0" ]; then
    ROS_STATUS="${GREEN}ALIVE${NC}"
fi

if [ -n "${ROS_DISTRO}" ]; then 
  echo -e "> ROS Version:\t\t\tROS $ROS_DISTRO ($ROS_DISTRO)"
fi
echo -e "> ROS Master:\t\t\t$ROS_MASTER_URI"
# Check optional ROS_IP param has been set or not
if [ -z ${ROS_IP+x} ]; then
    # Not set, so display message and path for setting
    echo -e "> ROS IP:\t\t\tNOT SET"
else
    # Set so display diagnostic
    echo -e "> ROS IP:\t\t\t$ROS_IP"
fi

echo -e "> Roscore Status:\t\t$ROS_STATUS"

echo "##########################################################################"
