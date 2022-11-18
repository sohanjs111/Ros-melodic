# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

LABEL key="value"

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
RUN apt update
RUN apt-get install -y ros-melodic-teleop-twist-keyboard

# pcl_ros
RUN apt install -y ros-melodic-pcl-ros

WORKDIR $ROS_WORKSPACE

# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

# making sure the file modes are executable
RUN chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py


WORKDIR $ROS_WORKSPACE/
COPY ./set-up-workspace /setup.sh
RUN sed -i -e 's/\r$//' /setup.sh
RUN chmod +x /setup.sh && /setup.sh 

## Hector Slam 
RUN apt update
RUN apt-get install -y ros-melodic-hector-slam
WORKDIR $ROS_WORKSPACE/src/
RUN mkdir -p aros/launch/
COPY ./launch/hectormapping_default.launch ./aros/launch/

## rf2o 
RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry $ROS_WORKSPACE/src/rf2o_laser_odometry
WORKDIR $ROS_WORKSPACE/src/
COPY ./launch/rf2o_laser_odometry.launch ./rf2o_laser_odometry/launch/

## Laser Scan 
RUN git clone https://github.com/CCNYRoboticsLab/scan_tools $ROS_WORKSPACE/src/scan_tools
RUN apt update
RUN apt-get install -y ros-melodic-csm
WORKDIR $ROS_WORKSPACE/src/
COPY ./launch/laser_scan_ma.launch ./scan_tools/laser_scan_matcher/demo/

## rqt-graph 
RUN apt update 
RUN apt install -y ros-melodic-rqt 
RUN apt install -y ros-melodic-rqt-graph 
RUN apt install -y ros-melodic-rqt-common-plugins

# Catkin_make 
RUN /setup.sh 

# xterm 
RUN apt update && apt install xterm

# add the evaluation file
WORKDIR $ROS_WORKSPACE/src/
RUN mkdir Rosbags
COPY ./evaluation ./evaluation
WORKDIR $ROS_WORKSPACE/src/evaluation/
RUN sed -i -e 's/\r$//' ./shell.sh

# install necessary packages for the shell file 
RUN apt update && apt install ros-melodic-tf2-geometry-msgs
RUN apt install bc 

WORKDIR $ROS_WORKSPACE/src/

# Catkin_make 
RUN /setup.sh && rm /setup.sh
