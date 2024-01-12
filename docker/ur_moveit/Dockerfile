# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

# NOTE: relies on the custom rosbase image
FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"
LABEL author "Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=noetic
ARG USER=qcr
ENV ROBOT_WS=/home/${USER}/robot_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# Install moveit for robotic control
RUN sudo apt-get update
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit -y

# Install all the official UR default configurations
RUN sudo apt-get install \
    ros-${ROS_DISTRO}-ur3-moveit-config \
    ros-${ROS_DISTRO}-ur3e-moveit-config \
    ros-${ROS_DISTRO}-ur5-moveit-config \
    ros-${ROS_DISTRO}-ur5e-moveit-config \
    ros-${ROS_DISTRO}-ur10-moveit-config \
    ros-${ROS_DISTRO}-ur10e-moveit-config \
    ros-${ROS_DISTRO}-ur16e-moveit-config \
    ros-${ROS_DISTRO}-ur20-moveit-config -y

# Workspace create and make for robot_ws - sets up the UR drivers for use 
# NOTE: overlaid on base ros noetic source
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone --branch master https://github.com/ros-planning/moveit_tutorials.git
RUN cd ${ROBOT_WS}/src && git clone --branch master https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
RUN cd ${ROBOT_WS}/src && git clone --branch noetic-devel https://github.com/ros-industrial/universal_robot.git
RUN cd ${ROBOT_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROBOT_WS} && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${ROBOT_WS}/devel/setup.bash" >> ~/.bashrc 

COPY entrypoint_setup.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]