#!/bin/bash
# Setup script for Space Concordia Development Environment

echo "Updating apt lists..."

sudo apt -qq update -y

echo "Setting up python virtual environment and installing requirements..."

python3 -m venv venv
source venv/bin/activate

sudo apt-get install -y python3-pip

pip install -U pip
pip install -r requirements.txt

python3 setup.py develop

echo "Installing ROS..."
bash scripts/install_ros.sh

echo "Installing Arduino IDE 2.0..."
bash scripts/arduino_install.sh

cp scripts/.bash_robotics ~/.bash_robotics
echo "source ~/.bash_robotics" >> ~/.bashrc
source ~/.bashrc

echo "Setting up Git hooks..."
cp commit_message_hook.py .git/hooks/prepare-commit-msg
cp branch_name_verification_hook.py ./git/hooks/post-checkout

echo "Install Workspace Dependencies"
sudo apt-get install -y ros-humble-pcl-msgs ros-humble-image-pipeline ros-humble-image-common ros-humble-vision-opencv ros-humble-perception-pcl ros-humble-ros2-ouster

echo "Building ROS Workspace"
cd robot/rospackages/
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
