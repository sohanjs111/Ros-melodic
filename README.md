# Ros-melodic

Docker Build
```
docker build -t ros-melodic .
```
Docker run the container 
```
docker run -dt --name ros-melodic -p 10000:10000 ros-melodic
```
To get into the container  
```
docker exec -it ros-melodic bash
```