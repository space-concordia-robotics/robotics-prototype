The first issue I came across is that `cdn.robotwebtools` doesn't work anymore, so any files linked to that domain don't work.
That said the files can be found at `static.robotwebtools`. We want to remove all external links to webpages so I've downloaded the files and placed them in `static/js/roslibjs`.

Another issue I found is that you need to connect to an http server in order to access the meshes used to visualize the robot in the GUI. The example links to the RobotWebTools website which doesn't seem to actually work. The solution is to host the meshes from the Flask app, so this was modified in the javascript. Here's some further reading:
https://github.com/RobotWebTools/roslibjs/issues/226
https://answers.ros.org/question/175164/cannot-load-mesh-marker-in-ros3djs/
https://answers.ros.org/question/61479/adding-robot_description-to-parameter-server/

To use the example as it used to be, I needed to install the pr2_description robot.
needed to download pr2_common from github in order to try using pr2_description because no kinetic version, make sure to deactivate venv because F
https://github.com/PR2/pr2_common

needed to install tf2 stuff, follow these instructions:
https://subscription.packtpub.com/book/hardware_and_creative/9781783554713/12/ch12lvl1sec105/installing-tf2-web-republisher-on-ros-kinetic

follow the commands recommended here to run stuff. some require venv to be deactivated:
http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF
1) `roslaunch pr2_description upload_pr2.launch`
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
