# robot_teleop_joy

***DISCLAIMER**: This has been tested on a wired XBOX360 controller. Results with others may vary.*  

Make sure to `cd` into the `rospackage` directory of the project repo (`robot/rospackages`) and run:
```
catkin_make
```
I think that's all you need to build the source code but you might need to tamper with `$ROS_PACKAGE_PATH`, but I don't remember (lol).

```
roslaunch robot_teleop_joy teleop_twist_joy.launch
```

This starts two nodes (`joy/joy_node` & `teleop_twist_joy/teleop_node`), both of which are nodes belonging to packages that were installed (i.e. they do not come from any custom code)! Yes, there is a python file inside `robot_teleop_joy` package inside the `scripts` folder called `robot_teleop_joy.py`, however, as it's written in the comments header (of that Python file), the file is merely used as an example file to show how mapping from `Joy` msgs to `Twist` msgs are done, however, this is already taken care for us by the `teleop_twist_joy/teleop_node` node, so we don't need that code. 

It is really important to echo the topics that are being written to in order to ensure the nodes are functioning properly. The two topics we want to echo are `/joy` and `/rover/cmd_vel`. This is how you do it:
```
$ rostopic echo /joy
```
Open another window, and enter:
```
$ rostopic echo /rover/cmd_vel
```

**SUPER IMPORTANT:** You will try to use the XBOX360 joystick to get feedback from the `rostopic echo` commands and you will notice IT DOES NOT WORK (yet the buttons seem to provide feedback). The solution: press the `A` button when using the joystick control. It's a built-in safety trigger so no one accidentally moves the joystick control.