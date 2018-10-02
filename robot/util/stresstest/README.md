# Odroid Stress Test

## Setup

If you don't already have a script where you keep your bash functions

Create one with: `touch ~/.bash_cmds.sh` (prepending the . makes it a hidden file)

Add the functions in the `.bash_cmds.sh` script

Make sure your ~/.bashrc sources ~/.bash_cmds.sh: append `. ~/.bash_cmds.sh` to your bashrc
Once you do this, you'll be able to call the bash function from your terminal (after sourcing ~/.bashrc, which happens everytime you open a new terminal). This has been tested to work both on the odroid running ubuntuMATE and on the space lab's ubuntu machine runing ubuntu desktop 16.04.

## Running the test

Run the stress test with: `./stress_test.sh`. It will default to run for 60 seconds.
It should print the cpu clock frequency and temperature every second, and once it's done the exit status and runtime.

You may specify how long to run the command for by simply adding the value in seconds as a first argument when to the script, so with `./stress_test.sh 10` it will run for 10 seconds.
