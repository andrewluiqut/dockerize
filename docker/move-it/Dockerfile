FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV MOVEIT_WS=/home/${USER}/moveit_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# install moveit
RUN sudo apt-get update
RUN sudo apt-get install ros-noetic-moveit -y

# install other packages

# workspace create and make 
RUN mkdir -p ${MOVEIT_WS}/src
RUN cd ${MOVEIT_WS}/src && \
    git clone https://github.com/ros-planning/moveit.git && \
    git clone https://github.com/ros-planning/moveit_tutorials.git -b master && \
    git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

RUN cd ${MOVEIT_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${MOVEIT_WS} && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# # copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${MOVEIT_WS}/devel/setup.bash" >> ~/.bashrc 

COPY assets/entrypoint_setup.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]
