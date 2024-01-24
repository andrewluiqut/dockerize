# Copyright 2024 - Andrew Kwok Fai LUI and Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

FROM moveit2
LABEL author "Andrew Lui <luia2@qut.edu.au>"
LABEL author "Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=humble
ARG USER=qcr
ENV ROBOT_WS=/home/${USER}/robot_ws
ENV PROJECT_WS=/home/${USER}/project_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# --- Install py-trees for behaviour functionality
RUN pip install py-trees==2.2.3

# --- Install required packages
RUN sudo apt-get update
RUN sudo apt-get install libpoco-dev libeigen3-dev \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-ros2-control-test-assets \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    python3-colcon-common-extensions -y

# --- Workspace create and make for robot_ws
# Enter the robot_ws and setup frankalib packages
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone https://github.com/frankaemika/libfranka.git --recursive
RUN cd ${ROBOT_WS}/src/libfranka && git checkout 0.13.2 
RUN cd ${ROBOT_WS}/src/libfranka && git submodule update --init
RUN mkdir ${ROBOT_WS}/src/libfranka/build
RUN cd ${ROBOT_WS}/src/libfranka/build && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ${ROBOT_WS}/src/libfranka
RUN cd ${ROBOT_WS}/src/libfranka/build && cmake --build . -j$(nproc)
RUN cd ${ROBOT_WS}/src/libfranka/build && cpack -G DEB
RUN cd ${ROBOT_WS}/src/libfranka/build && sudo dpkg -i libfranka-*.deb

# Install the franka_ros2 package in the robot_ws
RUN cd ${ROBOT_WS}/src && git clone https://github.com/frankaemika/franka_ros2.git

# Rebuild the workspace with updated packages
RUN cd ${ROBOT_WS} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Create and source the Project Workspace
# NOTE overlaid on the robot_ws build
RUN mkdir -p ${PROJECT_WS}/src
RUN cd ${PROJECT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source ${ROBOT_WS}/install/setup.bash && \
    cd ${PROJECT_WS} && \
    colcon build

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${ROBOT_WS}/install/setup.bash" >> ~/.bashrc 

COPY ros2_entrypoint.sh /usr/local/bin/ros_colcon_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_colcon_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_colcon_entrypoint"]
CMD ["bash"]