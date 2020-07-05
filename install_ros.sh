#!bin/bash
# Apache License 2.0
# Copyright (c) 2017, ROBOTIS CO., LTD.

# This script is adapted to be the Space Concordia Robotics Division needs
# Changes were made to not set up catkin_ws, update the bashrc edits, and remove the confirmation check

echo "[Set the target OS, ROS version"
name_os_version=${name_os_version:="bionic"}
name_ros_version=${name_ros_version:="melodic"}

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install the ros-desktop-full and all rqt plugins]"
sudo apt-get install -y ros-$name_ros_version-desktop-full ros-$name_ros_version-rqt-*

echo "[Initialize rosdep]"
sudo sh -c "rosdep init"
rosdep update

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Set the ROS environment]"
sh -c "echo \"\n\n#------ POST ROS INSTALLATION ------\" >> ~/.bashrc"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"\nalias cw='cd ~/Programming/robotics-prototype/robot/rospackages/'\" >> ~/.bashrc"
sh -c "echo \"alias cm='cw && catkin_make'\" >> ~/.bashrc"
sh -c "echo \"\nexport ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"


echo 'ROBOTICS_WS="/home/$USER/Programming/robotics-prototype"
BASE="$ROBOTICS_WS/robot/basestation"
ROVER="$ROBOTICS_WS/robot/rover"
ROSPACKAGES="$ROBOTICS_WS/robot/rospackages"
BASH_A="~/.bash_aliases"
NANORC="~/.nanorc"

# general shortcuts
alias ..="cd .."
alias b="cd -"
alias robotics="cd $ROBOTICS_WS"
alias base="cd $BASE"
alias rover="cd $ROVER"
alias arm="cd $ROVER/ArmDriverUnit"
alias wheels="cd $ROVER/MobilePlatform"
alias rostings="cd $ROSPACKAGES"
alias mcunode="cd $ROSPACKAGES/src/mcu_control/scripts"' >> ~/.bash_aliases

source $HOME/.bashrc

echo "[Complete!!!]"
exit 0
