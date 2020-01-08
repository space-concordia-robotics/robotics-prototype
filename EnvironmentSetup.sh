#!/usr/bin/env bash
# Setup script which install the Space Concordia Robotics Software team's development environment. 

APPEND_TO_BASH= "

#competition mode
#export ROS_MASTER_URI=http://172.16.1.30:11311
#export ROS_HOSTNAME=$USER

. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases

"

FINAL_MESSAGE="The script will now exit, you should test the installation using these steps:
1. test python using 'pytest'
2. verify ROS-Kinetic installation using 'roscore'
3. test GUI by running 'rosgui' and then 'startgui'
-> to see the GUI open a browser (preferably chrome) and go to localhost:5000"


## START

#check if in proper directory
if [ ! -f /home/tim/Programming/robotics-prototype/EnvironmentSetup.sh ]
then
   echo -e "\e[31m\e[47mYou did not setup the repo directory correctly. Refer to README\e[0m"
   exit 0
fi
   
   
#install prereqs
sudo apt install python-pip virtualenv git -y

sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update -y
sudo apt install python3.6 -y


# Setup venv
virtualenv -p `which python3.6` venv
source venv/bin/activate


# Install Requirements
pip install -r requirements.txt -r requirements-dev.txt


# Setup python and allow for module imports from within repo
python setup.py develop


# Check if ros is already installed, install it if it isn't
ROS_VERSION=$(rosversion -d)

if [ $ROS_VERSION = "kinetic" ]
then
    ROS_EXISTS=true
elif [ $(rosversion -d) = "rosversion: command not found" ]
then 
    echo "You do not have ROS installed, installing.."
    
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
    chmod 755 ./install_ros_kinetic.sh
    bash ./install_ros_kinetic.sh
    rm ./install_ros_kinetic.sh

    sudo apt install ros-kinetic-rosbridge-suite -y
else
    echo "A different ROS installation has been found... Please upgrade manually and rerun the script."
    exit 0
fi


# Install camera stuff
sudo apt-get install ros-kinetic-cv-camera ros-kinetic-web-video-server -y


# Build catkin
cd ~/Programming/robotics-prototype/robot/rospackages
catkin_make


# Edit ~/.bashrc
sudo echo "$APPEND_TO_BASH" >> ~/.bashrc


# Run env.sh
./robot/basestation/env.sh > robot/basestation/static/js/env.js


# Check if ffmpeg is already installed and install if not
if  ! dpkg-query -W -f='${Status}' ffmpeg | grep "ok installed"
then 
 sudo apt install ffmpeg -y
fi


# Setup git hooks
cd /home/$USER/Programming/robotics-prototype
cp commit-message-hook.sh .git/hooks/prepare-commit-msg
cp branch-name-verification.sh .git/hooks/post-checkout

# Exit
echo "$FINAL_MESSAGE"
exit 0