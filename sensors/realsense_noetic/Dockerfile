# Copyright 2024 - Dasun Gunasinghe
# Research Engineering Facility, Queensland University of Technology (QUT)

FROM rosbase 
LABEL author="Dasun Gunasinghe <dasun.gunasinghe@qut.edu.au>"

ARG ROS_DISTRO=noetic
ARG USER=qcr
ENV SENSOR_WS=/home/${USER}/sensor_ws
SHELL [ "/bin/bash", "-c" ]

# --- Install any other required packages
RUN sudo apt-get install ros-${ROS_DISTRO}-realsense2-camera ros-${ROS_DISTRO}-realsense2-description -y

# --- Create and source the Sensor Workspace
# NOTE: currently unpopulated for the realsense (as it is mostly apt installable)
RUN mkdir -p ${SENSOR_WS}/src
RUN cd ${SENSOR_WS} && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${SENSOR_WS} && \
    catkin_make
    
# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${SENSOR_WS}/devel/setup.bash" >> ~/.bashrc 

COPY ros1_entrypoint.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]