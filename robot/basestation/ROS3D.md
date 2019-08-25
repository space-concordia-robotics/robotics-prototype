# Dependencies

## Libraries
The first issue I came across is that `cdn.robotwebtools` doesn't work anymore, so any files linked to that domain don't work.
That said the files can be found at `static.robotwebtools`. We want to remove all external links to webpages so I've downloaded the files and placed them in `static/js/roslibjs`.

## ROS packages
Two packages needed to display robots in the GUI are `tf2_web_republisher` and `rosbridge_suite`. For `rosbridge_suite` you simply run `sudo apt-get install ros-kinetic-rosbridge-suite`. For tf2 republisher, follow the instructions in this link:
https://subscription.packtpub.com/book/hardware_and_creative/9781783554713/12/ch12lvl1sec105/installing-tf2-web-republisher-on-ros-kinetic

## Robot Descriptions
The example seems to need a launch file to load data into the `robot_description` parameter. It may also do other things, I'm not super familiar with launch files, but our custom robot seems to work just with `robot_description`.

To use the example the way it's explained online, I needed to install the `pr2_description` robot. To do so I downloaded `pr2_common` from github in order to try using `pr2_description` because I couldn't find a ROS kinetic version.
https://github.com/PR2/pr2_common

Eventually we switched to using a custom-made ROS package that was made as a simplified version of the arm. The package is called `colarm` and is located in rospackages. We needed to make/modify the launch file in order to get it displayed in the GUI. Basically, the launch file sets the `robot_description` ROS parameter to the contents of the urdf (or xacro) file. The specific line in the roslaunch file differs based on the file format, which we noticed when looking at the pr2 launch file since that robot uses xacro files and `colarm` uses a URDF file. Also, I'm not sure if I'm able to run `colarm.launch` with the `roslaunch colarm colarm.launch` command, it seemed as though I needed to use `roslaunch path/to/colarm.launch` instead. To be checked later.

# Displaying the robot in the GUI

## Instructions to set everything up
Follow the commands recommended here to run stuff. some require `venv` to be deactivated, otherwise I got strange errors:
http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF

1) if using pr2, `roslaunch pr2_description upload_pr2.launch`
OR
1) if using colarm, go to rospackages and `roslaunch colarm/colarm.launch`
2) `rosrun robot_state_publisher robot_state_publisher`
3) `rosparam set use_gui true`
4) `rosrun joint_state_publisher joint_state_publisher`
5) `rosrun tf2_web_republisher tf2_web_republisher`
6) `roslaunch rosbridge_server rosbridge_websocket.launch`
OR
6) `rosgui`
7) `startgui`

The next step would be to make a launch file which goes through steps 1 to 5. Steps 6 and 7 should be done separately, just like they currently are in `master`, because each command returns feedback that needs to be shown in separate terminals.

Also, someone should take note of which commands need to be run with `venv` deactivated. For example, launching the rosbridge websocket server needs this. Some research should be done to understand why `venv` must be deactivated.

## How we got the javascript to work properly
Another issue I found is that you need to connect to an http server in order to access the meshes used to visualize the robot in the GUI. The example links to the RobotWebTools website which doesn't seem to actually work. The solution is to host the meshes from the Flask app, so this was modified in the javascript. I started by pasting the `pr2_description` folder into `/static/` but afer we switched to using `colarm`, we eventually discovered how to have Flask serve the files from a different directory using some commands in `app.py`. So instead of having `colarm` in `rospackages` and having a second copy in `/static/` (the second one being used to access the meshes), flask will get the meshes from `rospackages`. Here's some further reading:
https://github.com/RobotWebTools/roslibjs/issues/226
https://answers.ros.org/question/175164/cannot-load-mesh-marker-in-ros3djs/
https://answers.ros.org/question/61479/adding-robot_description-to-parameter-server/
