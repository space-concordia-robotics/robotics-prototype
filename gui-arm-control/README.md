# gui-arm-control

### Current Tasks (phase 3)

Sprint 1:
- consult with motor-interface/arm-comms on expected data/format
- create design (even if just on paper) for the full gui panel.

Sprint 2:
- implement the code for the gui based of design, assume to fill with dummy variables for all panels, unless otherwise specified below.

```
Arm-Position panel: integrate max's matlab code. First start with workspace code, then inverse kinematics code.
```


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
    or if you develop in Python 2: `pacman -S mingw-w64-x86_64-python2-gobject` (for our GUI purposes we are currently using Python 2)
3. When the above is done, the set up is complete and:
- Python is installed
- GTK is installed
- Glade is installed
(all of which can be found in the default installation folder `C:\mysys64\mingw64\bin`)
4. To setup development the location for Python has to be added to your PATH environment variable.
Here's a [HOWTO](https://www.computerhope.com/issues/ch000549.htm) for different kinds of Windows.
In this case you need to add `C:\mysys64\mingw64\bin`.

#### Linux
1. open up your terminal
2. use your package manager to install the dependency needed to run a demo of the GUI with `python ArmGUILayout.py`:
- `sudo apt-get install python-gtk2`
- `sudo apt-get install libgtk-3-dev`
3. Optional development tools:
- `sudo apt-get install glade` - graphic interface designer
- `sudo apt-get install geany` - text editor

### OSX
Before getting started it is assumed that you have [HomeBrew](https://brew.sh/) installed and you have python 2.7 installed and linked to home brew.

1. Instsall
	- `$ brew install pygobject3 --with-python3 gtk+3`  

2. The Fix
	- makes sure the the installs you did with homebrew are recongized as environment variables
	- `$ brew reinstall pygobject pygobject3`  
	- `$ mkdir -p "$HOME/Library/Python/2.7/lib/python/site-packages"`  
	- `$ echo  'import site; site.addsitedir("/usr/local/lib/python2.7/site-packages")' >>` `"$HOME/Library/Python/2.7/lib/python/site-packages/homebrew.pth"`  

3. Create a Demo File  
	- Create a file called **hello.py** with the following demo content

	``` python
	import gi
	gi.require_version("Gtk", "3.0")
	from gi.repository import Gtk

	window = Gtk.Window(title="Hello World")
	window.show()
	window.connect("destroy", Gtk.main_quit)
	Gtk.main()
	```

4. Run
	- `$ cd to/your/directory/with/the/demo`  
	- `$ python2 hello.py`  
	- Voila! 😃  
	![screenshot](http://pygobject.readthedocs.io/en/latest/_images/start_macos.png)  

5. References  
	- pygobject [docs](http://pygobject.readthedocs.io/en/latest/	getting_started.html)  
	- the fix [link](https://github.com/jeffreywildman/homebrew-virt-manager/issues/73)  
	- 	Big thanks to [@bnduwi](https://github.com/bnduwi) for taking the time to show me how to get it to work.
