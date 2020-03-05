#!/usr/bin/env bash
# Setup script which installs the Space Concordia Robotics Software team's rover environment.

APPEND_TO_BASH="

. ~/Programming/robotics-prototype/venv/bin/activate
. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases

"

FINAL_MESSAGE="The script will now exit"

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
sudo apt install python3.6 python3.6-venv git nodejs npm -y

# Setup venv
cd $REPO
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
source /opt/ros/kinetic/setup.bash # Have to source this from catkin installation b/c catkin_make uses some aliases that need to be sourced before or it'll fail
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

# Setup systemd services
cd $REPO/robot/rover
sudo cp systemd/config-ethernet.service /etc/systemd/system/config-ethernet.service
sudo cp systemd/ip-emailer.service /etc/systemd/system/ip-emailer.service
sudo cp systemd/ros-rover-start.service /etc/systemd/system/ros-rover-start.service
sudo systemctl enable config-ethernet.service
sudo systemctl enable ip-emailer.service
sudo systemctl enable ros-rover-start.service

# Setup ethernet and emailer service scripts
cd $REPO/robot/util
sudo cp configEthernet/runConfigEthernet.sh /usr/bin/runConfigEthernet.sh
sudo cp emailer/runEmailer.sh /usr/bin/runEmailer.sh

# Exit
echo "$FINAL_MESSAGE"
exit 0
