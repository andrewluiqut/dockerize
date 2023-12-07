FROM rosbase
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]
ENV ARMER_WS=/home/${USER}/armer_ws
ENV ROS_MASTER=true

# install armer workspace
RUN mkdir -p ${ARMER_WS}/src
RUN cd ${ARMER_WS}/src && \ 
git clone https://github.com/qcr/armer_msgs

# install python modules

ADD src/arm_tutorial/requirements.txt .
#RUN pip install -r ./requirements.txt

# copy entry point script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]
