/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

/* Running the code
 launch roscore
 cmd: roscore
 run rosserial client application that forwards your arduino messages to the rest of ROS
 cmd: rosrun rosserial_python serial_node.py /dev/ttyUSB0 (or ACM0, whatever name of your arduino port is)
 To observe the messages being published
 cmd: rostopic echo chatter
*/

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
