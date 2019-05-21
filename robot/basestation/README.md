# Flask Prototype
This is the SpaceConcordia's Robotic web GUI using Flask.

To view the previous GUI please refer to the `AsimovOperation.py` file located [here](https://github.com/space-concordia-robotics/robotics-prototype/tree/asimov-operation-gui-flask-prototype-42/robot/archives/gtk).

## Dependencies
Please refer to the main [README](https://github.com/space-concordia-robotics/robotics-prototype) to the instructions on setting up virtualenv and installing
the projects depenencies.

## How to use
Run the app with `./app.py`

You will see: `Running on http://127.0.0.1:5000`

Visit this link in your browser to see the GUI.

If the rover machine is running `StreamDispatcher.py`, then you will see a live video stream of the arm camera.

You also need to have the rosbridge_suite package installed. After running app.py, run rosbridge_websocket.launch to connect the webpage to a ROS websockets server (port 9090 by default).
You may install it with `sudo apt install ros-kinetic-rosbridge-suite`.
If you run `roslaunch rosbridge_server rosbridge_websocket.launch` with the `venv` activated you will run into issues, for now the solution is to simply deactivate the `venv` in this terminal only.

