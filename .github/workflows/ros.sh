OS_VERSION=bionic
ROS_VERSION=melodic


echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install the ros and extra ros packages]"
sudo apt-get install -y ros-$ROS_VERSION-desktop-full ros-$ROS_VERSION-rqt-* ros-$ROS_VERSION-rosbridge-suite ros-$ROS_VERSION-cv-camera ros-$ROS_VERSION-web-video-server python-rosdep

echo "[Initialize rosdep and install dependencies]"
sudo sh -c "rosdep init"
rosdep update
cd robotics-prototype/robot/rospackages
rosdep install --from-paths src --ignore-src -r -y

# Build catkin
echo '[Build ros packages using catkin]'
source /opt/ros/$ROS_VERSION/setup.bash 
catkin_make

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$ROS_VERSION/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Complete!!!]"
exit 0


