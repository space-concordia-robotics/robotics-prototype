# ros2_aruco

ROS2 Wrapper for OpenCV Aruco Marker Tracking

This package depends on a recent version of OpenCV python bindings and transforms3d library:

```
pip3 install opencv-contrib-python transforms3d
```

## ROS2 API for the ros2_aruco Node

This node locates Aruco AR markers in images and publishes their ids and poses.

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `poll_delay_seconds` - how many seconds to wait between captures
* `camera_index` - which camera index to open (0 corresponds to /dev/video0)
* `camera_destination_index` - If present, will attempt to use v4l2loopback 
                                to allow another process to access the camera.
                                Needs ffmpeg and v4l2loopback-dev installed.

## Running Marker Detection

1. Using the launch file - parameters will be loaded from _aruco\_parameters.yaml_.
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
2. As a single ROS 2 node - you can specify parameter values at startup by adding `--ros-args -p marker_size:=.05`, for example.
```
ros2 run ros2_aruco aruco_node
```

## Generating Marker Images

```
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
