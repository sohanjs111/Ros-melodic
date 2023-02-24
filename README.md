# Ros-melodic
Ros melodic docker was installed as per the need of the Bachelor Thesis at FHWS for Evaluating the performance on LiDAR-based Algorithms.

If you would like to use the ros-melodic as a root user continue the installation on [master branch](#master). 
If you would like to use the ros-melodic as a normal user checkout to [irobot branch](#irobot)

To install simple Ros melodic Docker without LiDAR-Based Alogrithms, please git checkout [simple branch](#simple). NOTE THAT YOU WILL BE RUNNING AN USER (irobot). 

If you want to use rqt_graph or xterm use following the steps at [xterm section](#xterm). Its also important to build the container according to your IP_address in [Step 2](#docker-steps). 


## FHWS

### Docker Steps
First clone the repo and move into the repo. Then follow the following steps for installation
First time installation run Steps 1 and 2. Later on, Follow only steps 3 to 6 

1. Build Docker Image
```
docker build -t ros-melodic .
```
Before you run the conatiner, you need to check for your ip address and replace it with 'your_ip' 
2. Run the Docker container:
  
  a. Regular container (without volumes)
```
docker run -dt --name melodic -p 10000:10000 ros-melodic
```
  b. Recommended to create two volumes, 'Rosbags' and 'evaluation'
```
docker run -dt --name melodic -v Rosbags:/home/irobot/catkin_ws/src/Rosbags -v evaluation:/home/irobot/catkin_ws/src/evaluation/ -p 10000:10000 ros-melodic
```
The above volumes can be created on Docker Desktop, however, recommended to mount physical volumes with an actually path. This can be easy done by replacing it with actually path of the volume that is to be mounted.

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
### Xterm
For using rqt_graph and xterm, one needs to download [Xlaunch](https://sourceforge.net/projects/vcxsrv/) on for windows. Execute VcxSrv before running xterm or rqt_graph in the container. 
1. Exectue Xlaunch from program files  
![Exectue Xlaunch](https://github.com/sohanjs111/Ros-melodic/blob/master/Images/vcxsrv.PNG)

2. ![click next](https://github.com/sohanjs111/Ros-melodic/blob/master/Images/vcxsrv2.PNG)

3. Make sure to check "*Disable Access Control*".
![click next](https://github.com/sohanjs111/Ros-melodic/blob/master/Images/vcxsrv3.PNG)

4. Then click Finish. 

Now the xterm or rqt_graph should work
## Branches 
### Master 
NOTE THAT YOU WILL BE RUNNING A ROOT USER
```
git checkout Lidar
```
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard
* xterm
* rqt_graph
* Hector slam 
* Rf2o laser odometry
* Laser Scan Matcher 
* shell file to run all the lidar algo


### Simple 
NOTE THAT YOU WILL BE RUNNING AN USER called ***irobot***.
```
git checkout simple
```
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard
* xterm
* rqt_graph

### irobot
NOTE THAT YOU WILL BE RUNNING AN USER called ***irobot***.
```
git checkout irobot
```
It installs the following for ROS melodic Docker 
* ROS-TCP-Endpoint for connecting ROS with Unity 
* Teleop Twist Keyboard
* xterm
* rqt_graph
* Hector slam 
* Rf2o laser odometry
* Laser Scan Matcher 
* shell file to run all the lidar algo

#### Change the username 
In order to change the user name, go to Dockerfile (line 5)
```
ARG USERNAME=irobot
```
Replace the *irobot* with the username of your choice. 
