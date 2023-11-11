import struct
import robot.rospackages.src.mcu_control_python.mcu_control_python.definitions.CommsDataTypes as dt
from std_msgs.msg import String

pds_out_commands = [("estop", 0, 2*[dt.UINT8_SIZE]), ("ping", 1, []), ("enable_motors", 2, 2*[dt.UINT8_SIZE])]

# pds_in_commands not necessary for PDS, handled by adapter code