#!/bin/bash

echo "Updating Locale..."
sudo apt -qq install locales -y
sudo locale-gen en_CA en_CA.UTF-8
sudo update-locale LC_ALL=en_CA.UTF-8 LANG=en_CA.UTF-8
export LANG=en_CA.UTF-8
REPO=/home/$USER/Programming/robotics-prototype

echo "Adding ROS source..."
ROS_DISTRO_TO_INSTALL="humble"
sudo apt-get -qq install software-properties-common
sudo apt-get update -y
sudo add-apt-repository universe -y

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "Updating sources and upgrading system packages..."
sudo apt -qq update -y
sudo apt -qq upgrade -y

echo "Installing ROS... (will take a while)"
sudo apt-get install python3-rospkg -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

echo "Initialize rosdep and install dependencies"
sudo rosdep init
rosdep update
cd $REPO/robot/rospackages
rosdep install --from-paths src -y --ignore-src -y

# Check if ros was successfully installed
if [ -e /opt/ros/humble/setup.sh ]; then
	source /opt/ros/humble/setup.bash
fi
if [ ${ROS_DISTRO} = "${ROS_DISTRO_TO_INSTALL}" ]; then
	echo "Done installing ROS ${ROS_DISTRO}"
	printenv | grep -i ROS
else
	echo "Install Failed "
	exit 1
fi
