# Ros-melodic
Ros melodic docker was installed as per the need of the Bachelor Thesis at FHWS for Evaluating the performance on LiDAR-based Algorithms. 
To install simple Ros melodic Docker without LiDAR-Based Alogrithms, please switch to simple - branch. 

## FHWS

First clone the repo and move into the repo. Then follow the following steps for installation
First time installation run Steps 1 and 2. Later on, Follow only steps 3 to 6 

1. Build Docker Image
```
docker build -t ros-melodic .
```
Before you run the conatiner, you need to check for your ip address and replace it with 'your_ip' 
2. Run the Docker container 
```
docker run -dt --name melodic -e DISPLAY='your_ip':0.0 -p 10000:10000 ros-melodic
```
3. If the Docker conatiner is not running, Only then run the command below
```
docker start melodic
```
4. To get into the container  
```
docker exec -it melodic bash
```
5. To exit the container after use
  ```
  exit
  ```
6. To Stop the container after use
  ```
  docker stop melodic
  ```

## Master 
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard
* Hector slam 
* Rf2o laser odometry
* Laser Scan Matcher 

## Simple 
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard

## Lidar 
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard
* xterm
* Hector slam 
* Rf2o laser odometry
* Laser Scan Matcher 
* shell file to run all the lidar algo
