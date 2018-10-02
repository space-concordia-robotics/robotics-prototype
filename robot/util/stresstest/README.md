# Odroid Stress Test

## Setup (optional)

This setup is optional, if you wish to test the functions on your system console at any point in time after setting up.

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

## Running the test

IMPORTANT: For the case of the odroid, if you see that the cpu temperature is above 80 degrees celsius you should stop the process.

Run the stress test with: `./stress_test.sh`. It will run for 60 seconds by default.
It should print the cpu clock frequency and temperature every second, and once it's done the exit status and runtime of the process.

You may specify how long to run the command for by simply adding the value in seconds as a first argument when to the script, so with `./stress_test.sh 10` it will run for 10 seconds.
You may also exit early via `Ctrl + c`.

NOTE: Since this script intended to run on the odroid, which has ARM architecture, the method for getting the current clock frequency requires access to a file owned by root, and must therefore be ran with `sudo` (but does not on x86).

Example: `sudo ./stress_test.sh 10`
