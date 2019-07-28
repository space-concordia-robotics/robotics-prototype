# ROS packages

## ping_acknowledgement

### Description

The ping_acknowledgement package uses the `PingResponse` service defined in `srv` folder to allow the server to respond to a simple ping from the client. Any string can be sent in order to get a response.

### Setup

Assuming ROS is properly installed, you need to setup a catkin workspace, add the package into the src folder and build the package with `catkin_make`.

The simplest way to make use of these packages is to run `catkin_make` from the `rospackages` folder and run `. /home/path-to-repo/robot/rospackages/devel/setup.bash`, or you can add this line to your `~/.bashrc` file to have it be run every time you open a new terminal.

After having built the package and generated the all the source code, you may test the service by running the following commands (each time in a new terminal):

- Start ros master with `roscore`
- Start server node with `rosrun ping_acknowledgment ping_response_server.py`
- Send a request/obtain response using client node with `rosrun ping_acknowledgment ping_response_client.py "<msg>"`, where you are free to choose what
ever message you wish to send.

**OR:**
- Start server launch file with (starts both server node and ROS Master if none detected):
```
roslaunch ping_acknowledgment ping_acknowledgment_server.launch
```
- SStart client launch file with
```
roslaunch ping_acknowledgment ping_acknowledgment_client.launch ping_msg:="hello"
```

### References

The following tutorials (and those preceding them) were followed and modified to produce the code:
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Creating a ROS msg and src](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)
- [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

## task_handler

### Description

This package contains a server to be run on the Odroid and a client to be run anywhere. One can call it to open or close certain Python scripts such as `ArmNode.py` or `RoverCommandListener.py` (to be replaced by a rosified version). It can be called either with the client node or through the `app.py` GUI when `rosbridge_websockets` is active.

## mux_selector

### Description

This package contains a server to be run on the Odroid and a client to be run anywhere. One can call it to select a desired multiplexer channel on the Odroid PCB. This package was created because communications to microcontrollers is done over UART. Howevere, since only one UART is available, a multiplexer allows the same UART to communicate with multiple devices. This package's service will raise or lower two of the Odroid's digital logic pins in order to choose one of four mux channels. It can be called either with the client node or through the `app.py` GUI when `rosbridge_websockets` is active.

## mcu_control

### Description

#### ArmNode.py
This node simultaneously behaves as a publisher, subscriber and service. It serves as an interface between the arm Teensy and the GUI, so it must be run on the Odroid. It subscribes to `/arm_command` messages which do not necessarily need feedback, responds to `/arm_request` requests which have expected responses, and publishes `/arm_joint_states` messages based on incoming angle data from the Teensy. It can be run directly, through the task_handler, or through the `app.py` GUI when `rosbridge_websockets` is active.

#### IkNode.py

This package also contains `IkNode.py`, an inverse kinematics node which behaves as a publisher and subscriber. It imports `ik_calculator.py` - a custom-made inverse kinematics library - which is used whenever it catches a new message from the `/ik_command` topic it subscribes to. It decides what angles to direct the Teensy to based on the `/arm_joint_states` topic that it also subscribes to, and publishes the command to `/arm_command` so `ArmNode.py` will send it to the Teensy.

### Usage

The expected method of communication is through the `app.py` GUI when rosbridge_websockets is active, but it can also be controlled with command line ROS commands or with custom publishers and clients. For more info on `rosbridge_websockets`, see the readme in `/robot/basestation`

## serial_cmd

### Description

This package contains a server to run on the Odroid and a client to be run anywhere. One can call it to quickly open a serial connection to a device wired to the Odroid, send a message and then close the connection. It can be called either with the client node or through the `app.py` GUI when `rosbridge_websockets` is active. A caveat is that it currently has no way of returning feedback from the serial device. It is intended to be paired with `odroid_rx`.

## odroid_rx

### Description

This package contains a publisher to run on the Odroid and a subscriber to be run anywhere. The publisher opens up communication with a serial port and publishes any incoming data. The subscriber listens to the data and logs it to a text file.
