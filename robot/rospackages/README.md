# ROS packages

## ping_acknowledgement

### Description

The ping_acknowledgement package uses the `PingResponse` service defined in `srv` folder to allow the server to respond to a simple ping from the clien
t. Any string can be sent in order to get a response.

### Setup

Assuming ROS is properly installed, you need to setup a catkin workspace, add the package into the src folder and build the package with `catkin_make`.

If you wish to use an already existing catkin workspace, then skip the following steps:

- Go to your home folder: `cd`
- Create the catkin_ws with a src directory in it: `mkdir -p catkin_ws/src
- Go into the src folder: `cd catkin_ws/src`

In the `catkin_ws/src/` folder:

- Copy over the package, while preserving permissions: `cp -pr ~/path-to-robotics-rototype/robot/rospackages/ping_acknowledgment .`
- Go back to catkin_ws: `cd ..`
- Build the package: `catkin_make`
- After a successful build, before you carry on make sure that the current catkin workstation folder is in your `$ROS_PACKAGE_PATH` environemnt variable. Check with `echo $ROS_PACKAGE_PATH`. If it is not there then you should append the a line like `. ~/catkin_ws/` to your `~/.bashrc` file. To use the package in the current terminal run `. ~/.profile` or open a new terminal.

After having built the package and generated the all the source code, you may test the service by running the following commands (each time in a new terminal):

- Start ros master with `roscore`
- Start server node with `rosrun ping_acknowledgment ping_response_server.py`
- Send a request/obtain response using client node with `rosrun ping_acknowledgment ping_response_client.py "<msg>"`, where you are free to choose what
ever message you wish to send.

### References

The following tutorials (and those preceding them) were followed and modified to produce the code:
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Creating a ROS msg and src](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)
- [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

