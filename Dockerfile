# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

ARG USERNAME=irobot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
 
LABEL buildDate="18.11.2022"
LABEL version="3.0"
LABEL name="Sohan Saldanha"
LABEL maintainer="sohan.saldanha@student.fhws.de"

 
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0770 /etc/sudoers.d/$USERNAME
 

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
ENV ROS_WORKSPACE=/home/$USERNAME/catkin_ws
# Copy the packages
COPY ./ros_packages/ $ROS_WORKSPACE/src/


# Install teleop
RUN apt update
RUN apt-get install ros-melodic-teleop-twist-keyboard
 
# pcl_ros
RUN apt install -y ros-melodic-pcl-ros

USER $USERNAME
 
# ROS-TCP-Endpoint
WORKDIR $ROS_WORKSPACE
RUN sudo git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b v0.7.0
 
# making sure the file modes are executable
RUN sudo chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py
 
# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
 
# Give Permission
RUN sudo chown $USERNAME:$USERNAME -R /home/$USERNAME/
 
# catkin_make
COPY ./set-up-workspace /setup.sh
RUN sudo sed -i -e 's/\r$//' /setup.sh
RUN sudo chmod +x /setup.sh && /setup.sh

## Hector Slam 
RUN sudo apt update
RUN sudo apt-get install -y ros-melodic-hector-slam
WORKDIR $ROS_WORKSPACE/src/
RUN mkdir -p aros/launch/
COPY ./launch/hectormapping_default.launch ./aros/launch/

## rf2o 
RUN sudo git clone https://github.com/MAPIRlab/rf2o_laser_odometry $ROS_WORKSPACE/src/rf2o_laser_odometry
WORKDIR $ROS_WORKSPACE/src/rf2o_laser_odometry/
COPY ./launch/rf2o_laser_odometry.launch ./launch/
RUN sudo sed -i '295s/if (dcenter > 0.f)/if (std::isfinite(dcenter) \&\& dcenter > 0.f)/' ./src/CLaserOdometry2D.cpp
RUN sudo sed -i '319s/if (dcenter > 0.f)/if (std::isfinite(dcenter) \&\& dcenter > 0.f)/' ./src/CLaserOdometry2D.cpp

## Laser Scan 
RUN sudo git clone https://github.com/CCNYRoboticsLab/scan_tools $ROS_WORKSPACE/src/scan_tools
RUN sudo apt update
RUN sudo apt-get install -y ros-melodic-csm
WORKDIR $ROS_WORKSPACE/src/
COPY ./launch/laser_scan_ma.launch ./scan_tools/laser_scan_matcher/demo/



## rqt-graph 
RUN sudo apt update 
RUN sudo apt install -y ros-melodic-rqt 
RUN sudo apt install -y ros-melodic-rqt-graph 
RUN sudo apt install -y ros-melodic-rqt-common-plugins

# Catkin_make 
RUN /setup.sh 

# xterm 
RUN sudo apt update && sudo apt install -y xterm

# add the evaluation file
WORKDIR $ROS_WORKSPACE/src/
RUN mkdir Rosbags
RUN sudo git clone https://github.com/sohanjs111/evaluation.git $ROS_WORKSPACE/src/evaluation
WORKDIR $ROS_WORKSPACE/src/evaluation/
RUN sudo sed -i -e 's/\r$//' ./shell.sh

RUN sudo chown $USERNAME:$USERNAME -R /home/$USERNAME/catkin_ws/src/

# install necessary packages for the shell file 
RUN sudo apt update && sudo apt install -y ros-melodic-tf2-geometry-msgs
RUN sudo apt install -y bc 

WORKDIR $ROS_WORKSPACE/src/

# Catkin_make 
RUN /setup.sh 

ENV DISPLAY=host.docker.internal:0.0

# Catkin_make 
#RUN sudo /setup.sh && sudo  rm /setup.sh
 