This package includes two action client/action server demos.

Example One: Fibonacci Sequence

Keep in mind that you can use double tab to help autocomplete `rosrun` commands, this usually only works after sourcing the appropriate `setup.bash` file.

1) `cd` into `robotics-prototype/robot/demos/RosAction`, run `catkin_make`

2) Run `source devel/setup.bash` - this must be done each time you open a new terminal

3) Run `roscore`

4) In a new terminal run `rosrun actionlib_tutorials fibonacci_server.py` - this server will perform the action

5) In a new terminal run `rostopic echo fibonacci/feedback` - this will allow you to monitor the feedback

6) In a new terminal run` rosrun actionlib_tutorials fibonacci_client.py` - this client sends the action to the server

There are three topics that we care about:
/fibonacci/goal - this is how many numbers in the fibonacci sequence we want to solve for
For example if the goal is set to 5 (order=5, line 22) in fibonacci_client.py, then the action
will solve for the first 5 numbers in the fibonacci sequence

/fibonacci/feedback - this updates every time a new number is added to the sequence

/fibonacci/result - when the action is complete, this will display the result.
For example if goal is set to order=5, then result will get published to this topic in the form of sequence: [0, 1, 1, 3, 5]

Example Two: Dishwasher Action
1) Run steps 1 to 3 from `Example One: Fibonacci Sequence` above

2) In a new terminal, run `dishwasher_action dishwasher_server.py`

3) In a new terminal, run `rostopic echo dishwasher/feedback`

4) In a new terminal, run `dishwasher_action dishwasher_client.py`

******************************************************************
Important Note: before trying to echo one of these topics, make sure to source devel/setup.bash
in the catkin_ws you are running this action from.
******************************************************************

There are three topics that we care about:
/dishwasher/goal - this is how many dishes will be washed (it just sets how many times a for loop
will run).

/dishwasher/feedback - every 10 dishes, a percentage of (dishes_washed/dirty_dishes)*100.0 gets published to /feedback

/dishwasher/result - when all the dishes are clean and the action is complete, a message in the form of a string will get published to this topic.



Other Interesting Control Features:

If you open either one of the action_server.py files (fibonacci_server.py/dishwasher_server.py), you will find a line with rospy.Rate(1) or rospy.Rate(2). The number you pass into this function sets how many times the loop in your code will update per second. So 1 Hz or 2 Hz. I'm not sure what the limit is on this, I didn't play with it too much. Feel free to play with it to see how it affects your action.

You can supersede what action the server is running by running another action that will target the same server. As a basic example (maybe a little redundant):
-you run the dishwasher server
-you run the dishwasher client
-your mom starts washing your filthy dishes
-you run the dishwasher client again from a separate terminal before the first action is completed.
This second client request will cancel the first client action request, and thus become the current action that the server is running.

This is important because we can use this as a method for canceling actions, replacing actions, etc.
