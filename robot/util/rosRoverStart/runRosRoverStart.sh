#/usr/bin/env bash
# source the rovers catking_ws setup.bash and run the one ros launchfile to rule them all
bash -c "source /home/odroid/catkin_ws/devel/setup.bash && roslaunch /home/odroid/rosRoverStart/rover.launch"
