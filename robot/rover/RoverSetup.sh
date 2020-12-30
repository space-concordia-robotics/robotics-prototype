#!/usr/bin/env bash
# Script which setups the Space Concordia Robotics Software team's OBC.

REPO="/home/$USER/Programming/robotics-prototype"

FINAL_MESSAGE="The script will now exit, ensure that ros is install correctly and that
all of the rospackages are present. Reboot to run systemd services."

#install prereqs
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update -y
sudo apt install nodejs npm openssh-server -y
sudo ufw allow ssh

# Setup venv
cd $REPO

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
cd $REPO/robot/util/configEthernet && bash synchConfigEthernet.sh
cd $REPO/robot/util/emailer/ && bash syncEmailer.sh

# Exit
echo "$FINAL_MESSAGE"
exit 0
