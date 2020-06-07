import subprocess

def run_shell(cmd, args="", print_output=True):
    """Run script command supplied as string.

    Returns tuple of output and error.
    """
    cmd_list = cmd.split()
    arg_list = args.split()

    for arg in arg_list:
        cmd_list.append(str(arg))

    if print_output:
        print("arg_list:", arg_list)
        print("cmd_list:", cmd_list)

    process = subprocess.Popen(cmd_list, stdout=subprocess.PIPE)
    output, error = process.communicate()

    return output, error

def get_pid(keyword):
    cmd = "ps aux"
    output, error = run_shell(cmd)

    ting = output.decode().split('\n')

    for line in ting:
        if keyword in line:
            #print("FOUND PID:", line)
            words = line.split()
            print("PID:", words[1])

            return words[1]

    return -1

def append_to_file(filename, text):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(text.rstrip('\r\n') + content)
