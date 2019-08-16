# ROS packages

## setting up ROS and the workspace

Assuming ROS is properly installed, you need to setup a catkin workspace, add the package into the src folder and build the package with `catkin_make`.

The simplest way to make use of these packages is to run `catkin_make` from the `rospackages` folder and either run the following line from your terminal or add it to your `~/.bashrc` file to have it be run every time you open a new terminal:
```
. /home/path-to-repo/robot/rospackages/devel/setup.bash`
```

After that, you may need to install additional packages to get the following homemade packages working. But in a perfect world all the dependencies are already handled.

## ping_acknowledgement

### Description

This package was the first one to be written and thus serves as an example for all future packages. The ping_acknowledgement package uses the `PingResponse` service defined in `srv` folder to allow the server to respond to a simple ping from the client. Any string can be sent in order to get a response.

### Setup

You may test the service by running the following commands (each command in a new terminal):

- Start ros master node with `roscore`. This node always has to be running before anything else can function. (Exception: it's automatically started when using `roslaunch`, see a few lines below)
- Start server node with `rosrun ping_acknowledgment ping_response_server.py`
- Send a request/obtain response using client node with `rosrun ping_acknowledgment ping_response_client.py "<msg>"`, where you are free to choose what ever message you wish to send.

**OR:**
- Start server launch file with (starts both server node and ROS master if master node isn't running):
```
roslaunch ping_acknowledgment ping_acknowledgment_server.launch
```
- Start client launch file with
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

This package contains multiple ROS nodes both for the basestation and the Odroid related to communicating with microcontrollers (MCUs). It also contains example scripts which were used to test different types of ROS node scripts.

Notably, this package holds nodes for communicating with the arm, science, rover and PDS MCUs. It also holds inverse kinematics, navigation, and joystick controller nodes. See the package for more details.

### Usage

The expected method of communication is through the `app.py` GUI when rosbridge_websockets is active, but it can also be controlled with command line ROS commands or with custom publishers and clients. For more info on `rosbridge_websockets`, see the readme in `/robot/basestation`.

## serial_cmd

### Description

This package contains a server to run on the Odroid and a client to be run anywhere. One can call it to quickly open a serial connection to a device wired to the Odroid, send a message and then close the connection. It can be called either with the client node or through the `app.py` GUI when `rosbridge_websockets` is active. A caveat is that it currently has no way of returning feedback from the serial device. It is intended to be paired with `odroid_rx`.

## odroid_rx

### Description

This package contains a publisher to run on the Odroid and a subscriber to be run anywhere. The publisher opens up communication with a serial port and publishes any incoming data. The subscriber listens to the data and logs it to a text file.

## cv_camera

### Description
This node is not homemade. It's used to connect to USB cameras and publish their feed to ROS.

## web_video_server

### Description
This node is not homemage. It's used to host camera feeds in a server which can be accessed by a webpage.
