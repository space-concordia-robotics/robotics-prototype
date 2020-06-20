#!/usr/bin/env bash

# to start the gui
alias updateEnv="bash $BASE/env.sh >| $BASE/static/js/env.js"
alias roverenv=". $ROBOTICS_WS/venv/bin/activate"
alias startgui="updateEnv && roverenv && cd $BASE && python app.py"
# we have to deactivate venv for this launch file to not break, bug to be resolved eventually
alias rosgui="roverenv && deactivate && roslaunch rosbridge_server rosbridge_websocket.launch"
