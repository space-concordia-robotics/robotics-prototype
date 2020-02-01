# Displaying the robot in the GUI

## Display the Arm
Upon starting the GUI with `rosgui` and then `startgui`, these are the steps to see the arm.

1) Open a terminal with `venv` activated and run `roslaunch arm_vision.launch`
2) Open a terminal with `venv` **deactivated** and run `roslaunch arm_display.launch`

You can read more on the workings of arm vision template [here](http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF).

## Next Steps
Some research should be done to understand why `venv` must be deactivated.

## ROS packages
Two packages needed to display robots in the GUI are `tf2_web_republisher` and `rosbridge_suite`. For `rosbridge_suite` you simply run `sudo apt-get install ros-kinetic-rosbridge-suite`. Further reading for `tf2_web_republisher` [here](https://subscription.packtpub.com/book/hardware_and_creative/9781783554713/12/ch12lvl1sec105/installing-tf2-web-republisher-on-ros-kinetic)