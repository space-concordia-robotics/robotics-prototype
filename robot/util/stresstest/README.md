# Odroid Stress Test

## Running the stress test (Python)

If on x86 architecture, run the script with: `./odroid_stresser.py 10` for 10 seconds of runtime. If no runtime is specified, it will default to 60 seconds.
If on arm architecture, run the script with `sudo ./odroid_stresser.py`

You may stop the process at anytime with `Ctrl + c`.

There will be logfiles created in the same folder as the script for the reports printed to screen, each time the script is run.

Here is a sample output from running the script, on x86 architecture:

```
robotics@Tesla:~/Documents/Programming/robotics-prototype/robot/util/stresstest$ ./odroid_stresser.py 
Checking for dependencies...

stress: info: [2639] dispatching hogs: 4 cpu, 0 io, 0 vm, 0 hdd
CPU temp: 41.000 C
CPU freq: 2507.966 MHz
CPU temp: 49.000 C
CPU freq: 3192.717 MHz
CPU temp: 50.000 C
CPU freq: 3192.718 MHz
CPU temp: 51.000 C
CPU freq: 3192.718 MHz
CPU temp: 52.000 C
CPU freq: 3192.717 MHz
^C
Terminating stress test early

Temperature stats:
Min CPU temp: 41.000 C
Max CPU temp: 52.000 C
Avg CPU temp: 48.6 C

Clock speed stats:
Min CPU freq: 2451.913 MHz
Max CPU freq: 3192.718 MHz
Avg CPU freq: 3055.7672 MHz
```

## Bash Setup (optional)

The reason for this section being here is because this script was initially developed in bash, and then ported into python (and added extra functionality in python).

This setup is optional, if you wish to test the (slightly outdated) bash functions on your system console at any point in time after setting up.

Existing functions:

- get_arch
- get_cpu_temp
- get_cpu_freq
- print_cpu_temp
- print_cpu_freq
- print_cpu_temp_freq
- print_cpu_freq_temp

If you just wish to run the script itself, you can skip this setup.
If you want to test the functions but only temporarily, source the file with: `. ~/bash_cmds.sh`.

Copy the script to your home folder

- `cp bash_cmds.sh ~/`

Make sure your ~/.bashrc sources ~/bash_cmds.sh: append `. ~/bash_cmds.sh` to your `~/.bashrc` file

Once you do this, you'll be able to call the bash function from your terminal (after sourcing ~/.bashrc again, which happens everytime you open a new terminal). 

You may optionally prepend a '.' to `bash_cmds.sh` if you wish for it to be a hidden file.

## Running the test (Bash)

Run the stress test with: `./stress_test.sh`. It will run for 60 seconds by default.
It should print the cpu clock frequency and temperature every second, and once it's done the exit status and runtime of the process.

You may specify how long to run the command for by simply adding the value in seconds as a first argument when to the script, so with `./stress_test.sh 10` it will run for 10 seconds.
You may also exit early via `Ctrl + c`.

NOTE: Since this script intended to run on the odroid, which has ARM architecture, the method for getting the current clock frequency requires access to a file owned by root, and must therefore be ran with `sudo` (but does not on x86).

Example: `sudo ./stress_test.sh 10`
