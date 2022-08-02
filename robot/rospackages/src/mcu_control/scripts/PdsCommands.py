import struct
import rospy
import robot.rospackages.src.mcu_control.scripts.CommsDataTypes as dt
from std_msgs.msg import String

pds_out_commands = [("estop", 0, []), ("ping", 1, []), ("enable_motors", 2, 2*[dt.ARG_STRING])]

# pds_in_commands not necessary for PDS, handled by adapter code