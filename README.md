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
2. Run the Docker container 
```
docker run -dt --name ros-melodic -p 10000:10000 ros-melodic
```
3. If the Docker conatiner is not running, Only then run the command below
```
docker start ros-melodic
```
4. To get into the container  
```
docker exec -it ros-melodic bash
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
* Hector slam 
* Rf2o laser odometry
* Laser Scan Matcher 

## Simple 
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 

