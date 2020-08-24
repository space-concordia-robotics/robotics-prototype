#!/usr/bin/env bash
# Setup script which install the Space Concordia Robotics Software team's development environment.

APPEND_TO_BASH="

#------ ROBOTICS SETTINGS ------
#competition mode
#export ROS_MASTER_URI=http://172.16.1.30:11311
#export ROS_HOSTNAME=$USER

. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases

#------ POST ROS INSTALLATION ------
alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'
alias cw='cd ~/Programming/robotics-prototype/robot/rospackages/'
alias cm='cw && catkin_make'
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
"

APPEND_TO_BASH_ALIASES='
ROBOTICS_WS="/home/$USER/Programming/robotics-prototype"
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
alias mcunode="cd $ROSPACKAGES/src/mcu_control/scripts"'

FINAL_MESSAGE="

#################################
The script will now exit, you should test the installation using these steps:
1. Open a new terminal window to apply changes
-> it should automatically start with virtual env activated and you should be able to use aliases that you can lookup in your ~/.bashrc and ~/.bash_aliases files
2. Test python by executing 'pytest' while in the 'robotics-prototype' directory
3. verify ROS installation using 'roscore'
4. Test GUI by running 'rosgui' and then 'startgui'
-> to see the GUI open a browser (chrome) and go to localhost:5000
#################################"

REPO="/home/$USER/Programming/robotics-prototype"
ROS_VERSION="melodic"

# check if in proper directory
SCRIPT_LOCATION=$REPO/EnvironmentSetup.sh
if [ ! -f $SCRIPT_LOCATION ]
then
    # -e enables text editing, \e[#m sets a text colour or background colour. \e[0 ends the edit.
    echo -e "\e[31m\e[47mYou did not setup the repo directory correctly. Refer to README\e[0m"
    exit 1
fi


# install prereqs
sudo apt update -y
sudo apt install python3.6-venv git python3-pip -y

# necessary for `ifconfig` in env.sh
sudo apt install net-tools

# Setup venv
python3.6 -m venv venv
source venv/bin/activate


# Install Requirements
pip install -U pip
pip install -r requirements.txt -r requirements-dev.txt


# Setup python and allow for module imports from within repo
python setup.py develop


# Install ROS
bash install_ros.sh


# Edit ~/.bash_aliases
# Ensures that you can connect to someones else's ip to access GUI
# Add aliases to terminal
# Makes your terminal start in (venv)
echo "$APPEND_TO_BASH" >> ~/.bashrc
echo "$APPEND_TO_BASH_ALIASES" >> ~/.bash_aliases
source ~/.bashrc


# Run env.sh
cd $REPO/robot/basestation
./env.sh >| static/js/env.js


# Setup git hooks
cd $REPO
cp commit_message_hook.py .git/hooks/prepare-commit-msg
cp branch_name_verification_hook.py .git/hooks/post-checkout


# Install and setup arduino IDE + Teensyduino
bash install_arduino_teensyduino.sh


# Exit
echo "$FINAL_MESSAGE"
