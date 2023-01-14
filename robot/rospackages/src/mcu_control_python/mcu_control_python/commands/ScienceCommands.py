import struct
import robot.rospackages.src.mcu_control_python.mcu_control_python.definitions.CommsDataTypes as dt
from std_msgs.msg import String

# All handlers should start with handle_ , while unrelated functions should not.


# https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=1131090349

science_out_commands = [("next_test_tube", 42, []), ("go_to_test_tube", 43, [dt.ARG_UINT8])]


science_in_commands = []
