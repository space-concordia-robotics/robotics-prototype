#!/bin/bash
# Setup script which install the Space Concordia Robotics Software team's development environment. 

# Setup venv
virtualenv -p `which python3.6` venv
source venv/bin/activate

# Install Requirements
pip install -r requirements.txt -r requirements-dev.txt

# Setup python and test it
python setup.py develop

# Install ROS-Kinetic
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
chmod 755 ./install_ros_kinetic.sh
bash ./install_ros_kinetic.sh

sudo apt install ros-kinetic-rosbridge-suite

# Install camera stuff
sudo apt-get install ros-kinetic-cv-camera
sudo apt-get install ros-kinetic-web-video-server

# Build catkin
cd ~/Programming/robotics-prototype/robot/rospackages
catkin_make

# Edit ~/.bashrc
sudo echo "

#competition mode
#export ROS_MASTER_URI=http://172.16.1.30:11311
#export ROS_HOSTNAME=$USER

. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases

" >> ~/.bashrc

# Run env.sh
./robot/basestation/env.sh > robot/basestation/static/js/env.js

echo "The Script will now exit, you should test the installation using these steps:
1. test python using 'pytest'
2. verify ROS-Kinetic installation using 'roscore'
3. deactivate venv using 'deactivate'
4. verify rosbridge-suite using 'roslaunch rosbridge_server rosbridge_websocket.launch'
5. Finally try the GUI by running 'rosgui' in one terminal and then in another 
'base'
'python app.py'
6. Install ffmpeg if needed using 'sudo apt install ffmpeg'"

exit 0