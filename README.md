# Mobile Robotics Laboratory

## Dockerized version

Install [docker](https://www.docker.com/products/docker-desktop/) in your system then run

```bash
./build.sh
```
to build the container then
```bash
./run_unity.sh
```
to open the hub or
```bash
./run_container.sh
```


## Folder
- ###  Installation folder:
    - install_unityHub.sh: script to install Unity
    - install_mono.sh: script to install mono for intellisense in visual studio code
    - install_ros2_foxy.sh: script to install ROS2
- ### Turtlebot3UnityROS2 folder:
    - In Unity Hub open this project
    - Load turtlebot3 scene if there isn't
- ### ROS2Package 
    - Include all ROS packages
