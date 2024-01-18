# Copyright 2024 - Andrew Kwok Fai LUI, Robotics and Autonomous System Group,
# Research Engineering Facility, RI
# the Queensland University of Technology

ARG ROS_DISTRO=noetic

FROM osrf/ros:${ROS_DISTRO}-desktop-full
LABEL author "Andrew Lui <luia2@qut.edu.au>"

ARG USER=qcr
ENV USER=${USER}
ENV ROS_DISTRO=${ROS_DISTRO}

# update apt-get and add user 'qcr' grant sudo privilege
RUN apt-get update && \
    apt-get install -y sudo && \
    adduser --disabled-password --gecos "" qcr  && \
    echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER ${USER}
WORKDIR /home/${USER}
SHELL ["/bin/bash", "-c"]

# complete rosdep installation
RUN rosdep update

# to overcome the interactive menu for the terminator installation
RUN /usr/bin/yes | sudo apt-get install software-properties-common
RUN sudo add-apt-repository ppa:gnome-terminator
RUN sudo apt-get update
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
terminator \
tmux

# install base packages
RUN sudo apt-get update && sudo apt-get install -y \
lsb-release \
git \
openssh-server \
ca-certificates \
net-tools \
vim

RUN git config --global core.editor "vim"

RUN sudo apt-get install -y \
locales \
wget \
curl \
dbus \
htop \
figlet \
build-essential \
cmake \
gdb

# install python
RUN sudo apt-get install -y \
python3-pip \
python3-rosdep \
python3-setuptools

# install jypyter-ros for running ros in jupyter notebook
RUN pip install numpy==1.24.4 notebook==6.5.5 ipykernel jupyros

# install chrome
RUN wget -q https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
RUN sudo apt-get install -y ./google-chrome-stable_current_amd64.deb
RUN rm ./google-chrome-stable_current_amd64.deb

# set locales
RUN sudo locale-gen en_US.UTF-8  
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8

# configuration for terminator
RUN mkdir -p /home/${USER}/.config/terminator
COPY assets/terminator/terminator_config.txt /home/qcr/.config/terminator/config 
COPY assets/terminator/terminator_background_ref.png /home/qcr/.config/terminator/background_ref.png

# install the qcr banner
COPY assets/banner.sh /usr/local/bin/banner

# modify the login script
RUN echo "source banner" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# copy and define entry script
COPY ./entrypoint_setup.sh /usr/local/bin/ros_entrypoint
RUN sudo chmod +x /usr/local/bin/ros_entrypoint
ENTRYPOINT ["/usr/local/bin/ros_entrypoint"]
CMD ["bash"]

