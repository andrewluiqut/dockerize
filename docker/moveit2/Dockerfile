# Copyright 2024 - Andrew Kwok Fai LUI, Robotics and Autonomous System Group,
# Research Engineering Facility, RI
# the Queensland University of Technology

FROM ros2base
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG ROS_DISTRO=humble
ARG USER=qcr
ENV MOVEIT_WS=/home/${USER}/moveit_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# install moveit
RUN sudo apt-get update
RUN sudo apt-get install ros-${ROS_DISTRO}-moveit -y

# install py_trees, py-trees-msgs and rqt-py-trees are relevant in ROS 1 only
# RUN sudo apt-get install \
#     ros-${ROS_DISTRO}-py-trees \
#     ros-${ROS_DISTRO}-py-trees-msgs \
#     ros-${ROS_DISTRO}-py-trees-ros \
#     ros-${ROS_DISTRO}-rqt-py-trees -y

RUN pip install py-trees==2.2.3

# install other packages (robot specific) 
# UR drivers for ROS2: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
RUN sudo apt-get install ros-${ROS_DISTRO}-ur -y

# workspace create and make for moveit_ws 
RUN mkdir -p ${MOVEIT_WS}/src
RUN cd ${MOVEIT_WS}/src && \
    git clone https://github.com/ros-planning/moveit2_tutorials -b humble
RUN cd ${MOVEIT_WS}/src && \
    vcs import < moveit2_tutorials/moveit2_tutorials.repos

# build colcon workspace for the moveit2 tutorial
RUN sudo apt update 
RUN cd ${MOVEIT_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default
# using sequential compilation for small memory size
# this process will take 15 to 30 minutes
RUN cd ${MOVEIT_WS} && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --mixin release --executor sequential

# workspace create for cgras_moveit_ws
# RUN mkdir -p /home/${USER}/cgras_moveit_ws/src
# RUN cd /home/${USER}/cgras_moveit_ws && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     cd ${MOVEIT_WS} && \
#     catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# optional: using CycloneDDS as a middleware. 
# note: this makes all nodes started using this RMW incompatible with any other nodes not using Cyclone DDS
# recommended: https://moveit.ros.org/install-moveit2/binary/
# RUN sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc 

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${MOVEIT_WS}/install/setup.bash" >> ~/.bashrc 

COPY assets/entrypoint_setup.sh /usr/local/bin/ros_colcon_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_colcon_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_colcon_entrypoint"]
CMD ["bash"]
