# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

FROM ros2base
LABEL author "Andrew Lui <luia2@qut.edu.au>"
LABEL author "Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=humble
ARG USER=qcr
ENV ROBOT_WS=/home/${USER}/robot_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# Install moveit and other base packages 
RUN sudo apt-get update
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit -y

# workspace create and make for robot_ws 
# TODO: add the Universal Robotics Driver repository (and others) needed to run on real robot
# NOTE: the simulated version has its own docker image (needs investigation)
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone --branch humble https://github.com/ros-planning/moveit2_tutorials.git
RUN cd ${ROBOT_WS}/src && vcs import < moveit2_tutorials/moveit2_tutorials.repos

# build colcon workspace for the moveit2 tutorial
RUN sudo apt update 
RUN cd ${ROBOT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# using sequential compilation for small memory size
# this process will take 15 to 30 minutes
RUN cd ${ROBOT_WS} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --mixin release --executor sequential

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${ROBOT_WS}/install/setup.bash" >> ~/.bashrc 

COPY entrypoint_setup.sh /usr/local/bin/ros_colcon_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_colcon_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_colcon_entrypoint"]
CMD ["bash"]