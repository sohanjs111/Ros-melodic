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
RUN apt update && apt -y install nano
 
# Create Catkin_ws
ENV ROS_WORKSPACE=/home/$USERNAME/catkin_ws
# Copy the packages
COPY ./ros_packages/ $ROS_WORKSPACE/src/
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint -b v0.7.0

# Install teleop
RUN apt-get install ros-melodic-teleop-twist-keyboard
 
# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/irobot/catkin_ws/devel/setup.bash" >> ~/.bashrc
 
## rqt-graph
RUN apt update
RUN apt install -y ros-melodic-rqt
RUN apt install -y ros-melodic-rqt-graph
RUN apt install -y ros-melodic-rqt-common-plugins
 
# xterm
RUN apt update && apt -y install xterm
 
USER $USERNAME
 
WORKDIR $ROS_WORKSPACE
# making sure the file modes are executable
RUN sudo chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py
 
# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/irobot/catkin_ws/devel/setup.bash" >> ~/.bashrc
 
# Give Permission
RUN sudo chown $USERNAME:$USERNAME -R /home/$USERNAME/
 
# catkin_make
COPY ./set-up-workspace /setup.sh
RUN sudo sed -i -e 's/\r$//' /setup.sh
RUN sudo chmod +x /setup.sh && /setup.sh

EXPOSE 10000

ENV DISPLAY=host.docker.internal:0.0