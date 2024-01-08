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

if [ -n "${ROS_VERSION}" ]; then 
  echo -e "> ROS Version:\t\t\tROS $ROS_VERSION ($ROS_VERSION)"
fi

if [ -n "${ROS_DISTRO}" ]; then 
  echo -e "> ROS Distro:\t\t\tROS $ROS_DISTRO ($ROS_DISTRO)"
fi

if [ -n "${ROS_DOMAIN_ID}" ]; then 
  echo -e "> ROS Domain ID:\t\t\t $ROS_DOMAIN_ID "
fi

echo "##########################################################################"
