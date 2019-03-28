# Comms

This demo folder was made to contain files for testing the UDP based communication library (in /robot/comms) initially made for sending drive commands.

## ArmCommandListener.py

This script is meant to test using the `receive()` function provided by the communication library.
To execute the test:

1. Run `./app.py` (make sure that when instantiating the Connection object you set the IP to "127.0.0.1", port: "5000")
2. Run `./ArmCommandListener.py` (make sure that the instantiation of the Connection object is done the same as above)
3. Press keys: w,s,e,d,r,f,t,g,y,h to see the ArmCommandListener script respond to the commands you issue!
