#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the dishwasher_action action, including the
# goal message and the result message.
import dishwasher_action.msg

def dishwasher_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (DishwasherAction) to the constructor.
    client = actionlib.SimpleActionClient('dishwasher', dishwasher_action.msg.DishwasherAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = dishwasher_action.msg.DishwasherGoal(dirtyDishes=50)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A DishwasherResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dishwasher_client_py')
        result = dishwasher_client()
        print("Result:" + result.dishesWashed)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
