# ROS Arduino Blink LED

Essentially testing the code provided in the beginner ROS tutorials ([here](http://wiki.ros.org/rosserial_arduino/Tutorials/Blink))

## Setup

All you need is any arduino (with a working built in LED), a USB-B to USB-A cable, and a [computer capable of running ROS](http://wiki.ros.org/kinetic/Installation).
I installed ROS using the quick install script from the ROS programming book:

1. `wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh`
2. `chmod 755 ./install_ros_kinetic.sh`
3. `bash ./install_ros_kinetic.sh`

For setting up [Arduino IDE](https://www.arduino.cc/en/Main/Software), follow instructions [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

## Running the demo

Launch [roscore](http://wiki.ros.org/roscore) in a new terminal window:

- `roscore`

Next, run the rosserial client application that forwards your Arduino messages ti the rest of ROS.
Make sure to use the correct serial port:

- `rosrun rosserial_python serial_node.py /dev/ttyUSB0`

Finally, you can toggle the LED using [rostopic](http://wiki.ros.org/rostopic):

- `rostopic pub toggle_led std_msgs/Empty --once`
