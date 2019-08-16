# MCU Control

This package holds many scripts and nodes related to controlling and communicating with MCUs.

## MCU nodes

### ArmNode.py
This node simultaneously behaves as a publisher, subscriber and service.
It serves as an interface between the arm Teensy and the GUI, so it must be run on the Odroid.
It subscribes to `/arm_command` messages which do not necessarily need feedback, responds to `/arm_request` requests which have expected responses, and publishes `/arm_joint_states` messages based on incoming angle data from the Teensy.
It also publishes `/battery_voltage` and `/arm_feedback`, the latter being general feedback not belonging to a specific subject.
It can be run directly via SSH, through the task_handler, or through the `app.py` GUI when `rosbridge_websockets` is active.

### RoverNode.py
This node simultaneously behaves as a publisher, subscriber and service.
It serves as an interface between the rover Teensy and the GUI, so it must be run on the Odroid.
It subscribes to `/rover_command` messages which do not necessarily need feedback, responds to `/rover_request` requests which have expected responses, and publishes `/rover_joint_states` messages based on incoming speed data from the Teensy.
It also publishes `/battery_voltage` and `/rover_feedback`, the latter being general feedback not belonging to a specific subject.
On top of the above, it also publishes speed (linear and angular) feedback through `/rover_twist` and heading, longitude and latitude through `/rover_position`.
It can be run directly via SSH, through the task_handler, or through the `app.py` GUI when `rosbridge_websockets` is active.

### PdsNode.py
This node simultaneously behaves as a publisher, subscriber and service.
It serves as an interface between the rover Teensy and the GUI, so it must be run on the Odroid.
It subscribes to `/pds_command` messages and publishes to `/battery_temps` and `/wheel_motor_currents` messages based on incoming temperature and current data from the Teensy.
It also publishes `/battery_voltage` and `/pds_feedback`, the latter being general feedback not belonging to a specific subject.
It can be run directly via SSH, through the task_handler, or through the `app.py` GUI when `rosbridge_websockets` is active.

### ScienceNode.py

## Inverse Kinematics

### ikNode.py

### ik_calculator.py

## Navigation

### AntennaNode.py

### NavigationNode.py

### Nav_funs.py

## examples

## rosweb_demo
