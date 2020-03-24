#!/usr/bin/env python3

import sys
import time
import rospy
from mcu_control.msg import RoverGoal, RoverGoalList
from std_msgs.msg import String


def create_goal_callback(message):
    goal = RoverGoal(message.name, message.long, message.lat)
    goal_list.append(goal)


def delete_goal_callback(message):
    name = message.data
    for goal in goal_list:
        if goal.name == name:
            goal_list.remove(goal)


if __name__ == '__main__':
    node_name = 'goals_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub/service functionality')

    create_goal_sub_topic = 'create_goal'
    create_goal_sub = rospy.Subscriber(create_goal_sub_topic, RoverGoal, create_goal_callback)

    delete_goal_sub_topic = 'delete_goal'
    delete_goal_sub = rospy.Subscriber(delete_goal_sub_topic, String, delete_goal_callback)

    goal_list_pub_topic = 'goal_list'
    rospy.loginfo('Beginning to publish to "' + goal_list_pub_topic + '" topic')
    goal_pub = rospy.Publisher(goal_list_pub_topic, RoverGoalList, queue_size=1)
    rate = rospy.Rate(1)  # 1Hz

    goal_list = []

    try:
        while not rospy.is_shutdown():
            goal_pub.publish(goal_list)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    def shutdown_hook():
        rospy.logwarn('This node (' + node_name + ') is shutting down')
        # give ROS time to deal with the node closing (rosbridge especially)
        time.sleep(1)

    rospy.on_shutdown(shutdown_hook)
