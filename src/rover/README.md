## libraries
The libraries in `libraries` folder are needed to run the PidController sketch.
These libraries were obtained through the arduino IDE, you can click on sketch > include library > manage libraries to find them.
Simply searching for 'adafruit' and 'encoder' and choosing the most appropriate looking libraries from the list allowed the sketch to be succesfully compiled.
This method installs it in your arduino libraries folder.

If you wish to refer the `include` statments to look for your own relative path to the libraries, you must surround the libary name with double quotes.
So for example, to use the libraries in `libraries` folder in conjunction with this sketch you'd need to have:
`#include "libraries/Encoder.h"`


