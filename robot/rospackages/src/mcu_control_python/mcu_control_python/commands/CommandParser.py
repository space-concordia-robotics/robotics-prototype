from robot.rospackages.src.mcu_control_python.mcu_control_python.commands.ArmCommands import arm_out_commands
from robot.rospackages.src.mcu_control_python.mcu_control_python.commands.WheelsCommands import wheel_out_commands
from robot.rospackages.src.mcu_control_python.mcu_control_python.commands.PdsCommands import pds_out_commands
from robot.rospackages.src.mcu_control_python.mcu_control_python.commands.ScienceCommands import science_out_commands

ARM_SELECTED = 0
ROVER_SELECTED = 1
PDS_SELECTED = 2
SCIENCE_SELECTED = 3

out_commands = [arm_out_commands, wheel_out_commands, pds_out_commands, science_out_commands]

def parse_command(message):
    full_command = message.data.split(" ")
    if full_command is not None:
        command = full_command[0]
        args = full_command[1:]
        newArgs = []
        for arg in args:
            newArgs.append(float(arg))

        return command, newArgs
    return None, []

def get_command(command_name, deviceToSendTo):
    for out_command in out_commands[deviceToSendTo]:
        if command_name == out_command[0]:
            return out_command

    return None

class EmptyMock(object):
    """ Class which does nothing when any method is called.
        Used to stub gpio calls in CommsNode local mode.
    """
    def __getattr__(self, name):
        def _missing(*args, **kwargs):
            return None
        return _missing

emptyObject = EmptyMock()
