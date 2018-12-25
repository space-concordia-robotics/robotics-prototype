# Rover
The code that runs on the rover

## PidController.ino
NOTE: the Adafruit library most probably won't end up being used since we decided against using adafruit motorshields in the end,
although this sketch will be either heavily refactored or completely rewritten by operations and power squads for 3 stepper, 1 DC
and 2 servo motors, this sketch can still be used to test interfacing python/arduino using a PID system to control a more simpler
set of motors, such as positional (non-continous) servos.
The moving of motors and current sensor readings will be programmed by the power squad.
Both teams need to decide on common expected formats for data to be properly communicated over serial.

To obtain the libraries using Arduino IDE you can click on `sketch` > `include library` > `manage libraries` and search to find them.
Simply searching for 'adafruit' and 'encoder' and choosing the most appropriate looking libraries from the list is all you need to do.
This method installs it in your arduino libraries folder.

If you wish to refer the `include` statements to set your relative path to the libraries, you must surround the libary name with double quotes.

## StreamDispatcher.py
Currently this starts a stream via `start_stream.sh` which uses mjpgstreamer, more details are documented in the bash script itself.
To start the stream, assuming you already have a usb web camera connected, run `./StreamDispatcher.py`. Use `Ctrl + C` to stop the stream.

Assuming the stream is being dispatched, it can be accessed in any browser by visiting: `server_ip:8090/?action=stream`,
where `server_ip` is replaced with the actual ip address  of the server doing the streaming.
If you have started the StreamDispatcher, opening the Arm page will show the live of feed of the camera in the Arm Vision panel.
The rover ip address in `app.py` will have to be set to the correct value.

Example:
`#include "libraries/Encoder.h"`

## ROS packages

### ping_acknowledgement

#### Description

The ping_acknowledgement package uses the `PingResponse` service defined in `srv` folder to allow the server to respond to a simple ping from the client. Any string can be sent in order to get a response.

#### Setup

Assuming ROS is properly installed, you need to setup a catkin workspace, add the package into the src folder and build the package with `catkin_make`. For help with this see [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

After having built the package and generated the all the source code, you may test the service by running the following commands:

- Start ros master with `roscore`
- Start server node with `rosrun ping_acknowledgment ping_response_server.py`
- Send a request/obtain response using client node with `rosrun ping_acknowledgment ping_response_client.py "<msg>"`, where you are free to choose whatever message you wish to send.

#### Sources

The following tutorials (and those preceding them) were followed and modified to produce the code:
- [Creating a ROS msg and src](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)
- [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
