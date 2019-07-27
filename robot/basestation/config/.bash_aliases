ROBOTICS_WS="~/Programming/robotics-prototype"
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
alias mcunode="cd $ROSPACKAGES/src/mcu_control/scripts"
alias roverenv=". $ROBOTICS_WS/venv/bin/activate"

# edit thyself
alias editalius="nano $BASH_A"
alias alius=". $BASH_A"
alias editnani="nano $NANORC"
alias nani=". $NANORC"

# to start the gui
alias startgui="roverenv && cd $BASE && python app.py"
# we have to deactivate venv for this launch file to not break, bug to be resolved eventually
alias rosgui="roverenv && deactivate && roslaunch rosbridge_server rosbridge_websocket.launch"
