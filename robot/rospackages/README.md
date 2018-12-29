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

### References

The following tutorials (and those preceding them) were followed and modified to produce the code:
- [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Creating a ROS msg and src](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)
- [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
