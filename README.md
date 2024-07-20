### Setup
The code in this repo was built around ROS Humble. First [install that](https://docs.ros.org/en/humble/Installation.html).
Then, from this folder:
- I recommend you setup a [Python venv](https://docs.python.org/3/library/venv.html). See steps below.
- Install rosdep, colcon, catkin, and pip (if not already) (`sudo apt install python3-colcon-common-extensions catkin_pkg python3-pip python3-rosdep2`)
- Init and update rosdep (`sudo rosdep init && rosdep update`)
- Run rosdep so it installs packages: `rosdep install --from-paths src --ignore-src -r -y`. Enter your password when prompted.
- Install misc python deps (`pip install -r requirements.txt`)
- Install JetsonGPIO [from GitHub](https://github.com/pjueon/JetsonGPIO/blob/master/docs/installation_guide.md). This must be 
manually installed. The default options will work.
- Build: `colcon build --symlink-install --packages-skip usb_cam` (usb_cam can only compile on the Jetson).

#### Setup venv
Run `python3 -m venv ./space-env` from the robotics-orin folder.

So you will use this frequently, add the source to your ~/.bashrc: `echo "source ${PWD}/space-env/bin/activate" >> ~/.bashrc`.
That way it will run automatically.

#### Setting up ros2_aruco

##### Short version
Run these commands:
- `pip install transforms3d`
- Find where your python packages are installed (eg run the above command again) and source it in `~/.bashrc`. This line should
**look like** the following: `export PYTHONPATH="/home/marc/Programming/robotics-orin/space-env/lib/python3.10/site-packages:$PYTHONPATH"`
- If need to duplicate video: `sudo apt install ffmpeg v4l2loopback-dkms v4l2loopback-utils v4l-utils`

##### Long version
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