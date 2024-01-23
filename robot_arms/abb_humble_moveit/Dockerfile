# Copyright 2024 - Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

FROM moveit2
LABEL author="Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=humble
ARG USER=qcr
ENV PROJECT_WS=/home/${USER}/project_ws
ENV ROBOT_WS=/home/${USER}/robot_ws
SHELL [ "/bin/bash", "-c" ]

# -- Install py-trees for behaviour functionality
RUN pip install py-trees==2.2.3

# -- Install any other required packages
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-ros2-controllers \
    libpoco-dev -y

# -- Workspace create and make for robot_ws - sets up the ABB ROS2 drivers for use 
# NOTE: overlaid on base ros2 humble source
# NOTE: this workspace would already be available via the moveit2 base image (which includes the tutorials)
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone --branch humble https://github.com/PickNikRobotics/abb_ros2.git
RUN cd ${ROBOT_WS}/src && vcs import < abb_ros2/abb.repos
RUN cd ${ROBOT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROBOT_WS} && \
    colcon build 

# -- Create and source the Project Workspace
# NOTE overlaid on the robot_ws build
RUN mkdir -p ${PROJECT_WS}/src
RUN cd ${PROJECT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source ${ROBOT_WS}/install/setup.bash && \
    cd ${PROJECT_WS} && \
    colcon build
    
# -- Copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${PROJECT_WS}/install/setup.bash" >> ~/.bashrc 

COPY ros2_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]