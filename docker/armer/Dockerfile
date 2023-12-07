FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV ARMER_WS=/home/${USER}/armer_ws
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# install armer workspace
RUN mkdir -p ${ARMER_WS}/src
RUN cd ${ARMER_WS}/src && \ 
git clone https://github.com/qcr/armer.git && \
git clone https://github.com/qcr/armer_msgs

# add ros dependency
RUN cd ${ARMER_WS} && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} 

# workspace create and make 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
cd ${ARMER_WS} && \
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# install roboticstoolbox
RUN pip install roboticstoolbox-python
# install python modules
RUN pip install -r ${ARMER_WS}/src/armer/requirements.txt
RUN pip install --force-reinstall spatialmath-python==1.0.0

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${ARMER_WS}/devel/setup.bash" >> ~/.bashrc 

COPY assets/entrypoint_setup.sh /usr/local/bin/ros_catkin_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_catkin_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_catkin_entrypoint"]
CMD ["bash"]
