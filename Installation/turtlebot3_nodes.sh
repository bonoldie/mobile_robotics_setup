#!/bin/bash
echo "###########################"
echo "# Turtlebot3 installation #"
echo "###########################"
echo ""
echo ""


echo "Turtlebot3 node"

apt install -y \
  ros-foxy-cartographer \
  ros-foxy-cartographer-ros \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-dynamixel-sdk \
  ros-foxy-turtlebot3-msgs \
  ros-foxy-turtlebot3

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc

source $HOME/.bashrc

