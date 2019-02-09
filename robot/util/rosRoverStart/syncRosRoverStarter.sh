ROS_ROVER_START_DIR="/home/odroid/rosRoverStart"

if [ ! -d $ROS_ROVER_START_DIR ]; then
    mkdir $ROS_ROVER_START_DIR;
fi

cp ./* $ROS_ROVER_START_DIR
