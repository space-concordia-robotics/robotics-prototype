### Setup
**TL,DR**: Run the venv and compilation setups. If you want to run arm
and wheels controls, or use the Aruco node, there are additional steps.
Then, you can run a launch file. For instance, to run the simulation of
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
Then, from this folder (`robotics-prototype`):
- Install rosdep, colcon, catkin, and pip (if not already) (`sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep2`)
- Init rosdep (`sudo rosdep init`). An error of the form `ERROR: default sources list file already exists:` is
expected, if you've already installed rosdep.
- Go in the rospackages folder: `cd robot/rospackages`
- Update rosdep and install `rosdep update && rosdep install --from-paths src --ignore-src -r -y`. Enter your password when prompted.
- Run rosdep so it installs packages:- Install misc python deps (`pip install -r ../../requirements.txt`)
- Install JetsonGPIO [from GitHub](https://github.com/pjueon/JetsonGPIO/blob/master/docs/installation_guide.md). This must be 
manually installed. The default options will work.
- Build: `colcon build --symlink-install --packages-skip usb_cam` (usb_cam can only compile on the Jetson).
- Source this ros2 package by adding a command to your bashrc. You can do that by running this command:
`echo "source ${PWD}/install/local_setup.bash" >> ~/.bashrc`
- **Restart your terminal** so the above command runs and you can have access to the ros2 workspace.

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