# Copyright 2024 - Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

FROM moveit
LABEL author="Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=noetic
ARG USER=qcr
ENV PROJECT_WS=/home/${USER}/project_ws
ENV ROBOT_WS=/home/${USER}/robot_ws
SHELL [ "/bin/bash", "-c" ]

# --- Install py-trees for behaviour functionality
RUN pip install py-trees==2.2.3

# --- Install any other required packages
# NOTE: some of these packages require individual install
#       - had issues via rosdep
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit-servo -y

# --- Workspace create and make for robot_ws - sets up the xarm drivers for use 
# NOTE: overlaid on base ros noetic source
# NOTE: this workspace would already be available via the moveit2 base image (which includes the tutorials)
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone --branch master --recursive https://github.com/xArm-Developer/xarm_ros.git
# Update the xarm_ros repo
RUN cd ${ROBOT_WS}/src/xarm_ros && git pull && git submodule sync && git submodule update --init --remote
# Perform rosdep update and install dependents
RUN cd ${ROBOT_WS} && rosdep update && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y 
# Build all packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${ROBOT_WS} && \
    catkin_make 

# Create and source the Project Workspace
# NOTE overlaid on the robot_ws build
RUN mkdir -p ${PROJECT_WS}/src
RUN cd ${PROJECT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source ${ROBOT_WS}/devel/setup.bash && \
    cd ${PROJECT_WS} && \
    catkin_make
    
# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${PROJECT_WS}/devel/setup.bash" >> ~/.bashrc 

COPY ros1_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]