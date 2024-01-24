# Copyright 2024 - Andrew Kwok Fai LUI, Robotics and Autonomous System Group,
# Research Engineering Facility, RI
# the Queensland University of Technology

ARG ROS_DISTRO=noetic

FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV CGRAS_WS=/home/${USER}/cgras_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# install armer workspace
RUN mkdir -p ${CGRAS_WS}/src
# RUN cd ${CGRAS_WS}/src && \ 
# git clone git@github.com:Coral-Imaging/cgras_imaging_agent.git

# add ros dependency
RUN cd ${CGRAS_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 

# workspace create and make 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
cd ${CGRAS_WS} && \
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# install chrome
RUN wget -q https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
RUN sudo apt-get install -y ./google-chrome-stable_current_amd64.deb
RUN rm ./google-chrome-stable_current_amd64.deb

# install python modules
COPY assets/requirements.txt ${CGRAS_WS}/src/requirements.txt
RUN pip install -r ${CGRAS_WS}/src/requirements.txt
RUN rm ${CGRAS_WS}/src/requirements.txt

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${CGRAS_WS}/devel/setup.bash" >> ~/.bashrc 

ENTRYPOINT ["/usr/local/bin/ros_entrypoint"]
CMD ["bash"]
