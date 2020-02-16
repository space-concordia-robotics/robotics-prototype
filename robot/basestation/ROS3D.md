# Displaying the robot in the GUI

## Display the Arm
Upon starting the GUI with `rosgui` and then `startgui`, these are the steps to see the arm.

1) Open a terminal with `venv` activated and run `roslaunch astro_arm arm_vision.launch`
2) Open a terminal with `venv` **deactivated** and run `roslaunch astro_arm arm_display.launch`

You can read more on the workings of arm vision template [here](http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF).

## Next Steps
Some research should be done to understand why `venv` must be deactivated.

## ROS packages
One required package not included in the default install script is `tf2_web_republisher`, and to learn how to install it [click here] (https://subscription.packtpub.com/book/hardware_and_creative/9781783554713/12/ch12lvl1sec105/installing-tf2-web-republisher-on-ros-kinetic)
