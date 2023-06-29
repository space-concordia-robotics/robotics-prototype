# Embedded Toolchain
This document goes over the writing, compiling, flashing, and testing of the embedded code. 
The robotics team is using STM32 chips as their microcontrollers of choice. To develop for the chips we recommend using the Segger Embedded IDE.
It provides all the compiling, programming, and debugging functionality necessary to work with ARM MCUs.


## Tools
To get started with embedded development a few items are necessary. 
1. microcontroller on dev board or breakout board. 
2. board debugger/programmer such as st-link v2, j-link, or others.
3. Embedded IDE (Segger)
4. GDB compiled for multiple architectures (gdb-multiarch)
5. openOCD

## Writing and Compiling code
Writing the code can be done in any preferred environment. It is recommended to have an LSP for code linting and analysis as well as the different head files used for the board. Code should be clean and commented. Avoid using magic numbers (numbers without a name) since it makes reading the code much more difficult. 

Compiling the code with the IDE is simple, but we need to make sure that the project was setup correctly respective to the chip used.
1. Using the segger package manager, install the package for your MCU.
2. Create a new project and choose the MCU family used. 
3. In project properties, change the debugger to gdb and the debug server to OpenOCD.
4. Set the OpenOCD command to `openocd -f /usr/local/share/openocd/scripts/interface/stlink.cfg -f /usr/local/share/openocd/scripts/target/stm32f1x.cfg`. (The target cfg file changes based on your MCU.)
4. Set the debug server to autostart. 

## Uploading to the MCU
With the IDE properly setup, the project should be automatically uploaded to the MCU during debugging. However, we can also use the st-link to upload a binary to the chip without going through the IDE.

## Debugging
Debugging is done through the IDE. If the project setup was done correctly, you simple need to start the debugging and a dissassembly window aswell as a register window should appear. 
