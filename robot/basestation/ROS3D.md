tried fixing a bunch of html/js
apparently cdn.robotwebtools doesn't work anymore, static.robotwebtools does though

needed to install tf2 stuff, follow these instructions:
https://subscription.packtpub.com/book/hardware_and_creative/9781783554713/12/ch12lvl1sec105/installing-tf2-web-republisher-on-ros-kinetic

needed to download pr2_common from github in order to try using pr2_description because no kinetic version, make sure to deactivate venv because F
https://github.com/PR2/pr2_common

in the js need to specify a specific http server where the collada mesh files are hosted, here's some info about it:
https://github.com/RobotWebTools/roslibjs/issues/226
https://answers.ros.org/question/175164/cannot-load-mesh-marker-in-ros3djs/
https://answers.ros.org/question/61479/adding-robot_description-to-parameter-server/
in this case a server is hosted so all the meshes need to be copied into the basestation/static folder and alert

follow the commands recommended here to run stuff. some require venv to be deactivated:
http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF
roslaunch pr2_description upload_pr2.launch
rosrun robot_state_publisher robot_state_publisher
rosparam set use_gui true
rosrun joint_state_publisher joint_state_publisher
rosrun tf2_web_republisher tf2_web_republisher
roslaunch rosbridge_server rosbridge_websocket.launch
