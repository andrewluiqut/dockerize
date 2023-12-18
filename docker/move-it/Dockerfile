FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV MOVEIT_WS=/home/${USER}/moveit_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# install moveit
RUN sudo apt-get update
RUN sudo apt-get install ros-noetic-moveit -y

# install py_trees, py-trees-msgs and rqt-py-trees are relevant in ROS 1 only
# RUN sudo apt-get install \
#     ros-${ROS_DISTRO}-py-trees \
#     ros-${ROS_DISTRO}-py-trees-msgs \
#     ros-${ROS_DISTRO}-py-trees-ros \
#     ros-${ROS_DISTRO}-rqt-py-trees -y
RUN pip install py-trees==2.2.3

# install other packages (robot specific)
RUN sudo apt-get install ros-${ROS_DISTRO}-ur10-moveit-config -y

# workspace create and make for moveit_ws 
RUN mkdir -p ${MOVEIT_WS}/src
RUN cd ${MOVEIT_WS}/src && \
    git clone https://github.com/ros-planning/moveit_tutorials.git -b master
# RUN cd ${MOVEIT_WS}/src && \
#     git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel 

RUN cd ${MOVEIT_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${MOVEIT_WS} && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# workspace create for cgras_moveit_ws
RUN mkdir -p /home/${USER}/cgras_moveit_ws/src
RUN cd /home/${USER}/cgras_moveit_ws && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${MOVEIT_WS} && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# download universal robots
RUN cd /home/${USER} && \
    git clone https://github.com/ros-industrial/universal_robot.git

# download robot_movement_interface
RUN cd /home/${USER} && \
    git clone https://github.com/ros-industrial/robot_movement_interface.git

# install jypyter-ros for running ros in jupyter notebook
# RUN pip install jupyros

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${MOVEIT_WS}/devel/setup.bash" >> ~/.bashrc 

COPY assets/entrypoint_setup.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]
