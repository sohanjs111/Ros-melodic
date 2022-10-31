# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# install nano 
RUN apt update && apt install nano

# Create Catkin_ws 
ENV ROS_WORKSPACE=/home/catkin_ws
# Copy the packages
COPY ./ros_packages/ $ROS_WORKSPACE/src/
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b v0.7.0

# Install teleop
RUN apt-get install ros-melodic-teleop-twist-keyboard

WORKDIR $ROS_WORKSPACE

# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

# making sure the file modes are executable
RUN chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py

# install python 
# RUN apt update \
#     && apt install -y software-properties-common 
# RUN apt autoremove -y -f python
# RUN add-apt-repository ppa:deadsnakes/ppa -y
# RUN apt update && apt install -y python3.7

# install pip 
# RUN apt-get update 
# RUN apt-get install -y python3-pip 
# RUN apt-get install -y python3-tk 

# RUN pip3 install easydict 
# RUN pip3 install PySimpleGUIQt 
# RUN pip3 install matplotlib 
# RUN pip3 install setuptools 
# RUN pip3 install mplcursors 

# GUI 
# RUN git clone https://github.com/florianspy/locchallbench $ROS_WORKSPACE/src/locchallbench
# COPY ./GUI/ $ROS_WORKSPACE/src/locchallbench/gui/

WORKDIR $ROS_WORKSPACE/
COPY ./set-up-workspace /setup.sh
RUN sed -i -e 's/\r$//' /setup.sh
RUN chmod +x /setup.sh && /setup.sh 

## Hector Slam 
RUN apt-get install -y ros-melodic-hector-slam

## rf2o 
RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry $ROS_WORKSPACE/src/rf2o_laser_odometry

## Laser Scan 
RUN apt-get install -y ros-melodic-scan-tools

# Catkin_make 

RUN  /setup.sh && rm /setup.sh


