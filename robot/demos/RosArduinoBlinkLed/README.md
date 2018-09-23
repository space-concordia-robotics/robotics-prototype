# ROS Arduino Blink LED Demo

## Setup

All you need is any arduino (with a working built in LED), a USB-B to USB-A cable, and a [computer capable of running ROS](http://wiki.ros.org/kinetic/Installation).
I installed ROS using the quick install script from the ROS programming book:

1. `wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh`
2. `chmod 755 ./install_ros_kinetic.sh`
3. `bash ./install_ros_kinetic.sh`

For setting up [Arduino IDE](https://www.arduino.cc/en/Main/Software), follow instructions [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

Make sure you run the following commands to install rosserial libraries:
- `sudo apt-get install ros-kinetic-serial-arduino`
- `sudo apt-get install ros-kinetic-serial`

## Running the demo

### One machine

Launch [roscore](http://wiki.ros.org/roscore) in a new terminal window:

- `roscore`

Next, run the rosserial client application that forwards your Arduino messages ti the rest of ROS.
Make sure to use the correct serial port, in my case it was ttyACM0.
If you're not sure what yours is, connect your arduino and run `ls /dev/tty*`.
If you're still not sure, unplug the arduino, run the same command and compare what's missing.

- `rosrun rosserial_python serial_node.py /dev/ttyACM0`

Finally, you can toggle the LED using [rostopic](http://wiki.ros.org/rostopic):

- `rostopic pub toggle_led std_msgs/Empty --once`

Or can also run the python script `blinkLED.py`.

Now you should be good to run the python publisher script, which will toggle the LED when the you press 't', and exit the program on 'q':

- `./blinkLED.py`

NOTE: to run this script with no errors, you will most probably have to delete and recreate your virtual environment folder again for this to work.

[This link](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) was useful in figuring out how to this part done.

### Multiple machines

[This tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) explains the setup necessary for running on multiple machines.

I was able to test running on multiple machines over the same network by configuring the machines as such:

Laptop (master):
- ROS_MASTER_URI=http://localhost:11311
- ROS_HOSTNAME=`hostname`
 
Odroid (w/ Arduino):
- ROS_MASTER_URI:http://<Laptop IP>
- ROS_HOSTNAME=`hostname`

Since I wasn't able to ping `odroid` hostname from my laptop (and vice versa), I had to edit my `/etc/hosts` file for each device
to make sure the hostnames would resolve to their corresponding IP addresses.
[This article](https://bencane.com/2013/10/29/managing-dns-locally-with-etchosts/) explains in more detail about managing DNS locally with `/etc/hosts`.

Assuming you can now ping hostnames of each devices bi-directionally, to run the demo remotely:
- On your machine run `roscore`
- On the odroid run `rosrun rosserial_python serial_node.py /dev/ttyACM0`
- On your machine run `rostopic pub toggle_led std_msgs/Empty --once`, or `./blinkLED.py` for a slightly more interactive experience
