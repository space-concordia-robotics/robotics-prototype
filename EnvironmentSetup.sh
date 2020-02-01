#!/usr/bin/env bash
# Setup script which install the Space Concordia Robotics Software team's development environment. 

APPEND_TO_BASH="

#competition mode
#export ROS_MASTER_URI=http://172.16.1.30:11311
#export ROS_HOSTNAME=$USER

. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases

"

FINAL_MESSAGE="The script will now exit, you should test the installation using these steps:
1. restart the terminal for certain changes to apply
-> it will automatically start with virtual env activated and you will be able to use aliases that you can lookup in your ~/.bashrc and ~/.bash_aliases files
2. test python executing 'pytest' while in the 'robotics-prototype' directory
3. verify ROS-Kinetic installation using 'roscore'
4. test GUI by running 'rosgui' and then 'startgui'
-> to see the GUI open a browser (preferably chrome) and go to localhost:5000" 

REPO="/home/$USER/Programming/robotics-prototype"

## START

#check if in proper directory
SCRIPT_DIRECTORY=$REPO/EnvironmentSetup.sh
if [ ! -f $SCRIPT_DIRECTORY ]
then
   # -e enables text editing, \e[#m sets a text colour or background colour. \e[0 ends the edit.
   echo -e "\e[31m\e[47mYou did not setup the repo directory correctly. Refer to README\e[0m"
   exit 1
fi
   
   
#install prereqs
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update -y
sudo apt install python3.6 python3.6-venv git -y

# Setup venv
python3.6 -m venv venv
source venv/bin/activate

# Install Requirements
pip install -U pip
pip install -r requirements.txt -r requirements-dev.txt


# Setup python and allow for module imports from within repo
python setup.py develop


# Check if ros is already installed, install it if it isn't
ROS_VERSION=$(rosversion -d)
if [ $ROS_VERSION = "<unknown>" ] || [ $? != 0 ] # $? = 0 when previous command succeeds
then 
    echo "You do not have ROS installed, installing..."
    
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
    chmod 755 ./install_ros_kinetic.sh
    yes "" | bash ./install_ros_kinetic.sh
    rm ./install_ros_kinetic.sh
	
	source ~/.bashrc
	
    sudo apt install ros-kinetic-rosbridge-suite -y

elif [$ROS_VERSION != "kinetic"]
then
    echo "A different ROS installation has been found... Please uninstall and rerun the script."
    exit 1
fi


# Install camera stuff, these are not ros package dependecies and not installed with rosdep
sudo apt-get install ros-kinetic-cv-camera ros-kinetic-web-video-server -y


# Build catkin
source /opt/ros/kinetic/setup.bash
cd $REPO/robot/rospackages
rosdep install --from-paths src --ignore-src -r -y
catkin_make


# Edit ~/.bash_aliases
# Ensures that you can connect to someones else's ip to access GUI
# Add aliases to terminal
# Makes your terminal start in (venv)
echo "$APPEND_TO_BASH" >> ~/.bashrc
source ~/.bashrc


# Run env.sh
cd $REPO/robot/basestation
./env.sh > static/js/env.js


# Setup git hooks
cd $REPO
cp commit-message-hook.sh .git/hooks/prepare-commit-msg
cp branch_name_verification_hook.py .git/hooks/post-checkout


# Exit
echo "$FINAL_MESSAGE"
exit 0
