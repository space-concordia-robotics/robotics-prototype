### Setup
**TL,DR**: Run the venv and compilation setups. If you want to run arm
and wheels controls, or use the Aruco node, there are additional steps.
Then compile with `colcon build --symlink-install --packages-skip usb_cam`.
Now, you can run a launch file. For instance, to run the simulation of
IK, run `ros2 launch arm_ik local_ik.launch.py`. **NOTE**: VS Code's
integrated terminal causes issues with RVIZ, so I recommend you use
a traditional terminal emulator (Terminator or Terminal).

#### Setup venv
We highly recommend you setup a [Python venv](https://docs.python.org/3/library/venv.html).
Run `python3 -m venv ./space-env` from the `robotics-orin` folder. This will create a python
venv in the `space-env` folder. To make it always run on startup, run the following command
to add it to your bashrc: `echo "source ${PWD}/space-env/bin/activate" >> ~/.bashrc`

#### Setup to compile repo
The code in this repo was built around ROS Humble. First [install that](https://docs.ros.org/en/humble/Installation.html).
Then, from this folder:
- I recommend you setup a [Python venv](https://docs.python.org/3/library/venv.html). See steps below.
- Install rosdep, colcon, and pip (if not already) (`sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep`)
- Init and update rosdep (`sudo rosdep init && rosdep update`)
- Run rosdep so it installs packages: `rosdep install --from-paths robot/rospackages/src --ignore-src -r -y`. Enter your password when prompted.
Run this **from the robotics-prototype folder**
- Install misc python deps (`pip install -r requirements.txt`)
- Install JetsonGPIO [from GitHub](https://github.com/pjueon/JetsonGPIO/blob/master/docs/installation_guide.md). This must be manually installed. The default options will work.
- `cd robot/rospackages`
- If you want to use ZED2 stuff, [install CUDA](https://developer.nvidia.com/cuda-downloads) and [install the zed sdk](https://www.stereolabs.com/en-ca/developers/release). Pick the
`ZED SDK for JetPack 6.0 GA (L4T 36.3) 4.1 (Jetson Orin, CUDA 12.2)` package.
- Build. On the Jetson use this command: `colcon build --symlink-install --packages-skip usb_cam LidarSlam --cmake-args -DCMAKE_BUILD_TYPE=Release` (usb_cam can only compile on the Jetson, LidarSlam needs to be built separately). Run this **from the rospackages folder**.
If you are on your laptop, you'll need to add more packages to `--packages-skip`, such as `zed-ros2-wrapper`, and 
`outer-ros`

#### Setup venv
Run `python3 -m venv ./space-env` from the robotics-prototype folder. Then source it to activate it: `source space-env/bin/activate`. NOTE: it may be necessary to install a dependency: `sudo apt install python3.10-venv`.

So you will use this frequently, add the source to your ~/.bashrc: `echo "source ${PWD}/space-env/bin/activate" >> ~/.bashrc`.
That way it will run automatically.

#### Compile LidarSlam
Install nanoflann: `sudo apt-get install -y libnanoflann-dev`. Then, compile:
`colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release`
NOTE: do this from the `robot/rospackages` folder.

#### Setup the lidar
Install additional packages:
```
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2
```
```
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
```
Then, the lidar package will compile.


Make sure that the ethernet interface the Lidar is set to is set to `link-local only`. This can be done within the 
ubunutu GUI. The [Networking guide](https://static.ouster.dev/sensor-docs/image_route1/image_route2/networking_guide/networking_guide.html)
can help, but this should be be only configuration that's necessary. The network config scripts **should**
not be necessary anymore. To test if it's on the network, run the following command:
`ping -c1 os1-992005000098.local`. Then, to *test* if it launches, run this command **in robot/rospackages**:
`ros2 launch ouster_ros driver.launch.py  params_file:='src/beep_autonomy/config/ouster_driver_params.yaml'`
To configure the network from a terminal (without GUI), use the `sudo nmtui` command.

### Running Autonomy
Run `ros2 launch beep_autonomy video.launch.py`

### Running arm and wheels controls
First, run `sudo ./scripts/configure-can0.sh` and `sudo ./scripts/configure-arm.sh`. The 
error message "chmod: cannot access '/dev/ttyUSB1': No such file or directory" is normal.
Then, run `ros2 launch robot/rospackages/launch/robot_ik.py`. NOTE: you will likely
need to unplug and re-plug in the USB-Serial adapter for the absolute encoders.

Then, on another computer, run `ros2 run joy joy_node` that is **on the same network**
as the rover, and with a Logitech X3D joystick plugged in (allowing a wider range
of input methods is in progress).

### Setting up ros2_aruco

#### Short version
Run these commands:
- `pip install transforms3d`
- Find where your python packages are installed (eg run the above command again) and source it in `~/.bashrc`. This line should
**look like** the following: `export PYTHONPATH="/home/marc/Programming/robotics-orin/space-env/lib/python3.10/site-packages:$PYTHONPATH"`
- If need to duplicate video: `sudo apt install ffmpeg v4l2loopback-dkms v4l2loopback-utils v4l-utils`

#### Long version
To get this working, you may see the following error:
```
Installing the transforms3d library by hand required. Please run
        sudo pip3 install transforms3d
```
Run this command (in my case without the sudo and with pip) and it should work.
Adding to PYTHONPATH is likely necessary. You will add a command like the following to `~/.bashrc`: 
`export PYTHONPATH="/home/marc/Programming/robotics-orin/space-env/lib/python3.10/site-packages:$PYTHONPATH"`

Another thing to consider is that while python3-opencv (installed with apt, from rosdep) works with
this package, it also works with opencv-python (installed through pip). There is a version difference,
which is why there are lines such as `if cv2.__version__ < "4.7.0":` in the source.

In addition, to allow the camera used for aruco detection to also be streamed, v4l2loopback must be configured
and ffmpeg must be present. Install the following: `v4l2loopback-dkms`, `v4l2loopback-utils`, `v4l-utils`, and `ffmpeg`.

If you have secure boot enabled, there will be additional setup involved (requiring a reboot). These packages might be added to `package.xml` 
at some point, but it will need to be added to rosdep, which takes a little while.

To allow simultaneous streaming on another process, set the `camera_index` and `camera_destination_index` params in the launch file.
They correspond to which index in `/dev/video` to use. If camera_destination_index is set (to a value which is not -1), it will
(attempt) to stream the camera from the source to the destination location (in `/dev/video`). Now, any other process can
load the camera from the destination index.