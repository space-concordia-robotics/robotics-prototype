# Embedded Toolchain
This document goes over the writing, compiling, flashing, and testing of the embedded code. 
The robotics team is using STM32 chips as their microcontrollers of choice. To develop for the chips we have different approaches. 
The first, easiest approach is using the STMCube IDE, which will makes peripheral configuration very easy, but adds a lot of boilerplate code and limits functionality. 
The second and third approaches are the Keil IDE and the Eclipse Embedded CDT. 
Keil offers a more complete suite of debugging tools but does not provide a good compiler.
Eclipse has a more customisable compiler but lacks in the debugging features and user friendliness.
The final approach is using the Arduino IDE, which is not recommended since we are trying to move away from the arduino ecosystem.

Noteworthy debugging solution [here](https://elrobotista.com/en/posts/stm32-debug-linux/)

## Tools
To get started with embedded development a few items are necessary. 
1. microcontroller on dev board or breakout board. 
2. board debugger/programmer such as st-link v2, or an arduino running as ISP
3. IDE which has code stepping and register viewing/manipulation during debug mode

## Writing and Compiling code
Writing the code can be done in any preferred environment. It is recommended to have an LSP for code linting and analysis as well as the different head files used for the board. Code should be clean and commented. Avoid using magic numbers (numbers without a name) since it makes reading the code much more difficult. 

To compile the code, you will either need to use the gcc toolchain or the included compiler with your IDE.
1. When using the IDE compiler, make sure that you have the project correctly setup for the chip you are programming. Otherwise, your program may not map correctly in the MCUs memory. 
2. When using the GCC toolchain, use the provided loader script to make sure the program gets loaded into the right memory regions of the binary. 

## Uploading to the MCU
Programming the MCUs requires a JTAG or st-link pogrammer. st-link is the recommended tool of choice since it comes with some binaries for ease of flashing, reading, and erasing MCUs. A JTAG programmer requires some more setup, but works fine as well.

## Debugging
The programmer also serves as the debugger interface. To debug, you will need the Keil, Eclipse, or any other tool that can disassemble, step through code, and show you/allow editing common registers. Without these features, it will be extremely difficult to debug programs. Generally, the debuggers will use OpenOCD along with the programmer to interface with the chip. 
