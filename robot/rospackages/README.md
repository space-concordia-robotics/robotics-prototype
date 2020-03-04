# ROS packages

## Setting up ROS and the workspace for our custom packages

Assuming ROS is properly installed, you need to have a catkin workspace in which all the packages are stored. Lucky for you, the `rospackages` folder in this repo is made specifically for this purpose. If this is your first time using one of these packages or you have just made modifications to a package, run `catkin_make` from the `rospackages` folder to build all the packages in this folder for use by ROS. Next, run the following line from your terminal, replacing `path-to-repo` with... the path to the repo on your computer:
```
. path-to-repo/robot/rospackages/devel/setup.bash
```
Add this line to your `~/.bashrc` file (to have it called every time you open a new terminal) if you haven't already, because otherwise your shell won't be aware of the nodes and other resources from packages in `rospackages`.

After that, you may need to install additional packages to get the homemade packages working. But in a perfect world all the dependencies are already handled.

## Setting up ROS and the workspace for isolated testing

If you would like to test a package in isolation from the rest of our packages, you can make your own workspace following a guide on the ROS wiki. To make sure it's fully isolated you can also comment out the `. path-to-rospackages/devel/setup.bash` line in `~/.bashrc` before opening new terminals. As usual, once your package is ready to be tested, run `catkin_make` in the workspace and then use the same strategy of sourcing `devel/setup.bash` except this time the file is in the isolated workspace.

## Useful bash commands
A very convenient way to test ROS nodes is with `rosbash` commands. For example, one can check which nodes are running using `rosnode list`, and for a more general idea, one can check which topics currently exist with `rostopic list`. One can listen in on a particular topic using `rostopic echo <topic-name>` and publish to a topic using `rostopic pub -1 <topic-name> <message-type> <message>`. For example, I commonly use something like `rostopic pub -1 /rover_command std_msgs/String 'hi'`. Tab completion makes this more convenient as `rosbash` is capable of guessing the message type based on the topic and can even autofill a template message, particularly useful for more complex message types like `sensor_msgs/JointState`. Similar commands can be used for sending requests to services or changing parameters.

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

Notably, this package holds nodes for communicating with the arm, science, rover and PDS MCUs. It also holds inverse kinematics, navigation, antenna, and joystick controller nodes. See the package for more details.

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

This package creates an opencv camera feed. The feed is automatically detected by a web_video_server node.

***Main parameters that we care about***
device_id: Used to select the camera that the node will use. Int values 0, 1, 2 since we have 3 cameras in total
image_width: specifies camera image width in pixels
image_height: specifies the camera image height in pixels

***How to set the parameters***
These parameters can be set on startup. For example:
rosrun cv_camera cv_camera_node _device_id:=0 _image_width:=1920 _image_height:=1080

Otherwise you can set the parameters before running the node:
rosparam set /cv_camera/device_id 1
rosparam set /cv_camera/image_width 1920
rosparam set /cv_camera/image_height 1080

***Running more than one cv_camera node***
First, understand that the default cv_camera node publishes the video feed to the topic /cv_camera/image_raw

Your first node can be run like normal with or without the parameters set at startup:
rosrun cv_camera cv_camera_node device_id:=0 image_width:=1920 image_height:=1080
and it will publish to /cv_camera/image_raw

any node after that must be run like this with or without launch parameters:
rosrun cv_camera cv_camera_node __name:=new-name
the new node will now publish to /new-name/image_raw

if you set the parameters before launch, here is what it would look like:
rosparam set /new-name/device_id 1
rosparam set /new-name/image_width 1920
rosparam set /new-name/image_height 1080

Here is a little example for clarification:
rosparam set /cv_camera2/device_id 1
rosparam set /cv_camera2/image_width 960
rosparam set /cv_camera2/image_height 720
rosrun cv_camera cv_camera_node __name:=cv_camera2

this will now publish to /cv_camera2/image_raw

No matter how many nodes you create, web_video_server will be able to access these nodes automatically.

### Description
This node is not homemade. It's used to connect to USB cameras and publish their feed to ROS.

## web_video_server

### Description
This node is not homemade. It's used to host camera feeds in a server which can be accessed by a webpage.
