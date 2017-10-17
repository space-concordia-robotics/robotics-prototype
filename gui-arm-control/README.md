## gui-arm-control

### Current Tasks

- Create a detailed list of final features
- consult with motor-interface/arm-comms on expected data/format

### Set up
For development you need to have Python, GTK+3, and Glade installed.
Glade is a graphic interface designer used to generate GUI layout easily and then later program modifications according to needs.
The easiest way to have all the required setup done is to follow the instructions found [here](https://www.gtk.org/download/windows.php) for windows, or [here](https://prognotes.net/2015/12/installing-gtk-3-and-glade-development-tools-in-linux/) for linux.

#### Windows
1. Download MYSYS2 [here](http://www.mysys2.org/). Run the installer.
2. Once MYSYS2 is installed run the following commands:
- `pacman -S mingw-w64-x86_64-gtk3`
- `pacman -S mingw-w64-x86_64-glade`
- `pacman -S mingw-w64-x86_64-devhelp`
- If you develop in Python 3: `pacman -S mingw-w64-x86_64-python3-gobject`
    or if you develop in Python 2: `pacman -S mingw-w64-x86_64-python2-gobject` (for our GUI purposes we are using Python 2)
3. When the above is done, the set up is complete and:
- Python is installed
- GTK is installed
- Glade is installed 
(all of which can be found in the default installation folder `C:\mysys64\mingw64\bin`)
4. To setup development the location for Python has to be added to your PATH environment variable.
Here's a [HOWTO](https://www.computerhope.com/issues/ch000549.htm) for different kinds of Windows.
In this case you need to add `C:\mysys64\mingw64\bin`.


