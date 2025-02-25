#!/bin/bash
echo "###########################"
echo "# ROS 2 Foxy installation #"
echo "###########################"
echo ""
echo ""

echo "Setting up repository and key"

apt update 
apt install curl gnupg2 lsb-release -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update

echo ""
echo "Installing ROS 2"
echo ""
apt install ros-foxy-desktop -y
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

echo ""
echo "Installing Colcon"
echo ""
apt install python3-colcon-common-extensions -y
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

echo ""
echo "ROS 2 installed"
echo ""
echo "Create ROS 2 environment"
echo ""

mkdir -p ~/colcon_ws/src


