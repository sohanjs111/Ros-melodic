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

COPY ./set-up-workspace /setup.sh
RUN sed -i -e 's/\r$//' /setup.sh
RUN chmod +x /setup.sh && /setup.sh 

WORKDIR $ROS_WORKSPACE

# Source the workspace on sign in
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

## rqt-graph 
RUN apt update 
RUN apt install -y ros-melodic-rqt 
RUN apt install -y ros-melodic-rqt-graph 
RUN apt install -y ros-melodic-rqt-common-plugins

# xterm 
RUN apt update && apt install xterm

# making sure the file modes are executable
RUN chmod +x src/ros_tcp_endpoint/src/ros_tcp_endpoint/*.py

# Catkin_make 
RUN /setup.sh 
