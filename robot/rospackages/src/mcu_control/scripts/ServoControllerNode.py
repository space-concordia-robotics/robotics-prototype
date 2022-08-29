#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from collections import deque

control_queue = deque()

SERVO_RANGE_MIN = 0
SERVO_RANGE_MAX = 180
current_servo_pos = 0

MOVE_SERVO_COMMAND = "move_servo"

def main():

    try:
        while not rospy.is_shutdown():
            manage_queued_control_commands()

    except KeyboardInterrupt:
        print("Node shutting down due to shutting down node.")

def manage_queued_control_commands():
    if (len(control_queue) > 0):
        servo_control = control_queue.popleft()

        servo_id = servo_control[0]
        servo_move_by = servo_control[1]

        global current_servo_pos

        new_servo_pos = current_servo_pos + int(servo_move_by)

        if new_servo_pos > SERVO_RANGE_MAX:
            current_servo_pos = SERVO_RANGE_MAX
        elif new_servo_pos < SERVO_RANGE_MIN:
            current_servo_pos = SERVO_RANGE_MIN
        else:
            current_servo_pos = new_servo_pos

        servo_command = MOVE_SERVO_COMMAND + " " + servo_id + " " + str(current_servo_pos)

        servo_command_Pub.publish(servo_command)

def servo_control_callback(message):
    rospy.loginfo('received: ' + message.data + ' command')
    message_split = message.data.split(" ")

    servo_id = message_split[0]
    servo_move_by = message_split[1]

    temp_struct = [servo_id, servo_move_by]
    control_queue.append(temp_struct)



if __name__ == '__main__':
    node_name = 'servo_controller_node'
    rospy.init_node(node_name, anonymous=False) # only allow one node of this type
    rospy.loginfo('Initialized "'+node_name+'" node for pub/sub/service functionality')

    servo_command_pub_topic = '/rover_command'
    rospy.loginfo('Beginning to publish to "'+servo_command_pub_topic+'" topic')
    servo_command_Pub = rospy.Publisher(servo_command_pub_topic, String, queue_size=10)

    servo_control_topic = '/servo_control'
    rospy.loginfo('Beginning to subscribe to "'+servo_control_topic+'" topic')
    sub = rospy.Subscriber(servo_control_topic, String, servo_control_callback)

    main()


