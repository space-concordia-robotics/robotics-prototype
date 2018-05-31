## PidController.ino
NOTE: the Adafruit library most probably won't end up being used since we decided against using adafruit motorshields in the end,
although this sketch will be either heavily refactored or completely rewritten by operations and power squads for 3 stepper, 1 DC
and 2 servo motors, this sketch can still be used to test interfacing python/arduino using a PID system to control a more simpler
set of motors, such as positional (non-continous) servos.
The moving of motors and current sensor readings will be programmed by the power squad.
Both teams need to decide on common expected formats for data to be properly communicated over serial.

To obtain the libraries using Arduino IDE you can click on `sketch` > `include library` > `manage libraries` and search to find them.
Simply searching for 'adafruit' and 'encoder' and choosing the most appropriate looking libraries from the list is all you need to do.
This method installs it in your arduino libraries folder.

If you wish to refer the `include` statements to set your relative path to the libraries, you must surround the libary name with double quotes.

Example:
`#include "libraries/Encoder.h"`

