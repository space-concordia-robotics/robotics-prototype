1) create your catkin_ws

2) cd into your catkin_ws/src

3) generate your ROS package (for your action)
Example package: catkin_create_pkg dishwasher_action actionlib message_generation roscpp rospy std_msgs actionlib_msgs

--Note that the following parameters are the dependences for your new action package--
actionlib message_generation roscpp rospy std_msgs actionlib_msgs

I recommend you include all of these when making action packages (they may not all be required, you're welcome to test this yourself)

4) create directory catkin_ws/src/dishwasher_action/action then cd into it

5) create dishwasher.action file. The .action file has the following 3 variables:
-goal: your input on what the action should complete e.g. a GPS coordinate, a distance to move, etc.
-feedback: updates on how the action is being performed e.g. rotated x degrees, moved 5 meters, 10% complete, need more lemon pledge, etc.
-result: a message/number indicating the result of the action: 100%, completed, failed, etc.

see dishwasher.action for exact syntax. Usable datatypes can be found at:
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

6) cd into your dishwasher_action directory and open CMakeLists.txt. Do the following:

use the add_action_files macro to declare the actions you want to be generated:

add_action_files(
  DIRECTORY action
  FILES Dishwasher.action
)

call the generate_messages macro, not forgetting the dependencies on actionlib_msgs and other message packages like std_msgs:

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

add actionlib_msgs to catkin_package macro like this:

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

catkin_package also specifies only CATKIN_DEPEND to actionlib_msgs. The transitive dependency on message_runtime is happening automatically.

7) Open package.xml and add "<exec_depend>message_generation</exec_depend>" to the end of other <exec_depend> parameters

8) cd to your catkin_ws and run catkin_make


