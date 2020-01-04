import subprocess

def run_shell(cmd, args=""):
    """Run script command supplied as string.
    Returns tuple of output and error.
    """
    cmd_list = cmd.split()
    arg_list = args.split()

    print("arg_list:", arg_list)

    for arg in arg_list:
        cmd_list.append(str(arg))

    print("cmd_list:", cmd_list)

    process = subprocess.Popen(cmd_list, stdout=subprocess.PIPE)
    output, error = process.communicate()

    return output, error

def get_pid(keyword):
    cmd = "ps aux"
    output, error = run_shell(cmd)

    ting = output.decode().split('\n')

    #print(ting)

    for line in ting:
        if keyword in line:
            #print("FOUND PID:", line)
            words = line.split()
            print("PID:", words[1])

            return words[1]

    return -1
