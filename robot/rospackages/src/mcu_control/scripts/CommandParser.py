from robot.rospackages.src.mcu_control.scripts.ArmCommands import arm_out_commands
from robot.rospackages.src.mcu_control.scripts.WheelsCommands import wheel_out_commands
from robot.rospackages.src.mcu_control.scripts.PdsCommands import pds_out_commands

ARM_SELECTED = 0
ROVER_SELECTED = 1
PDS_SELECTED = 2
SCIENCE_SELECTED = 3

out_commands = [arm_out_commands, wheel_out_commands, pds_out_commands, None]

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