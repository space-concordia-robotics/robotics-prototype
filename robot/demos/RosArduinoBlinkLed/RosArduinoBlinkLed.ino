/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

/* Running the code
 launch roscore
 cmd: roscore
 run rosserial client application that forwards your arduino messages to the rest of ROS
 cmd: rosrun rosserial_python serial_node.py /dev/ttyUSB0 (or ACM0, whatever name of your arduino port is)
 To observe the messages being published
 cmd: rostopic pub toggle_led std_msgs/Empty --once
*/

#include <ros.h>
#include <std_msgs/Empty.h>
int BUILTIN_LED = 13;

ros::NodeHandle nh;

// must take a constant refrence of a message as argument
void messageCallback(const std_msgs::Empty& toggle_msg) {
    digitalWrite(BUILTIN_LED, HIGH-digitalRead(BUILTIN_LED)); // blink the LED
}

// instantiate subscriber with name toggle_led and attach callback function to it
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCallback);

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);
    // initialize node handle, allowing to create publishers/subscribers and taking care of serial port comms
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
  // handle passing messages to the subscriber callback
  nh.spinOnce();
  delay(1);
}
