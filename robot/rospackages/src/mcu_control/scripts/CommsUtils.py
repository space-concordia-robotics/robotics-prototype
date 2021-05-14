def parse_command(message):
    full_command = message.split(" ")
    if full_command is not None:
        command = full_command[1]
        args = full_command[2:]
        newArgs = []
        for arg in args:
            try:
                newArgs.append(float(arg))
            except ValueError:
                newArgs.append(arg)

        return command, newArgs
    return None, []


def get_arg_bytes(command_tuple):
    return sum(element[1] for element in command_tuple[2])

