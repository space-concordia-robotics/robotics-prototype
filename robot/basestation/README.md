# Flask Prototype
This is Space Concordia Robotics Division's web GUI using Flask.

## Dependencies
Please refer to the main [README](https://github.com/space-concordia-robotics/robotics-prototype) to the instructions on setting up virtualenv and installing the projects depenencies.

You also need to have the rosbridge_suite package installed. After running app.py, run rosbridge_websocket.launch to connect the webpage to a ROS websockets server (port 9090 by default).
You may install it with `sudo apt install ros-kinetic-rosbridge-suite`.

## How to use
In the config folder you will find `.bash_aliases` with the aliases `rosgui` and `startgui` that can be run in two terminals. Otherwise, follow the instructions in the next paragraph.

Open a terminal and type `deactivate` (otherwise the ROS websocket server will crash, for now the solution is to simply deactivate the `venv` in this terminal only). Then type `roslaunch rosbridge_server rosbridge_websocket.launch`. In a new terminal (make sure `venv` is activated here), run the GUI with `./app.py`.

Make sure it works fine: after running the `./app.py` go visit `localhost:5000` in your browser (recommended browser: Google Chrome).
