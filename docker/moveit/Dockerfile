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

# Create the robot workspace and build the moveit tutorials
# NOTE: this is where specific robot functionality packages will go (i.e., ros drivers, etc)
RUN mkdir -p ${ROBOT_WS}/src
RUN cd ${ROBOT_WS}/src && git clone --branch master https://github.com/ros-planning/moveit_tutorials.git
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