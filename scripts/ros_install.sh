#!/bin/bash

echo "Updating Locale..."
sudo apt -qq install locales -y
sudo locale-gen en_CA en_CA.UTF-8
sudo update-locale LC_ALL=en_CA.UTF-8 LANG=en_CA.UTF-8
export LANG=en_CA.UTF-8

echo "Adding ROS source..."
sudo apt-get -qq install software-properties-common
sudo apt-get update -y
sudo add-apt-repository universe

sudo apt -qq install curl
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Updating sources and upgrading system packages..."
sudo apt -qq update
sudo apt -qq upgrade -y

echo "Installing ROS... (will take a while)"
sudo apt -qq install ros-humble-desktop
sudo apt -qq install ros-dev-tools

echo "Done installing ROS"
