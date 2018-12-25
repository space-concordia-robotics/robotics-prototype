# ROS packages

## ping_acknowledgement

### Description

The ping_acknowledgement package uses the `PingResponse` service defined in `srv` folder to allow the server to respond to a simple ping from the clien
t. Any string can be sent in order to get a response.

### Setup

Assuming ROS is properly installed, you need to setup a catkin workspace, add the package into the src folder and build the package with `catkin_make`.
Steps:
- Go to your home folder: `cd`
- Create the catkin_ws with a src directory in it: `mkdir -p catkin_ws`
- Go into the src folder: `cd catkin_ws/src`
- Copy over the package, while preserving permissions: `cp -r ~/path-to-robotics-prototype/robot/rospackages/ping_acknowledgment .`
- Go back to catkin_ws: `cd ..`
- Build the package: `catkin_make`
- After succesfull build, for the package to be added to your $ROS_PACKAGE_PATH environment variable: `. devel/setup.bash`. If you want this to happen everytime you open a new terminal, then append this command (use absolute or relative path accordingly) to your `~/.bashrc` file.

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

