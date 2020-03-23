#!/usr/bin/env python3

import sys
import time
import rospy
from mcu_control.msg import RoverGoal, RoverGoalList


def subscriber_callback(message):
    goal = RoverGoal(message.name, message.long, message.lat)
    goal_list.append(goal)


if __name__ == '__main__':
    node_name = 'goals_node'
    # only allow one node of this type
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo('Initialized "' + node_name +
                  '" node for pub/sub/service functionality')

    goal_sub_topic = 'create_goal'
    goal_sub = rospy.Subscriber(goal_sub_topic, RoverGoal, subscriber_callback)

    goal_pub_topic = 'goal_list'
    rospy.loginfo('Beginning to publish to "' + goal_pub_topic + '" topic')
    goal_pub = rospy.Publisher(goal_pub_topic, RoverGoalList, queue_size=1)
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
