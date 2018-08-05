# gui-arm-control
Graphic User Interface (GUI) for controlling the arm, and receiving relevant info about the arm and it's environment. The GUI is developed using [PyGobject](http://pygobject.readthedocs.io/en/latest/index.html) a Python package which provides bindings for GObject based libraries such as GTK+, GStreamer, WebKitGTK+, GLib, GIO. The library that will be used to develop our Python GUI application will be be GTK+.

### Set up
For development you need to have PyGobject installed, (includes [GTK+](https://www.gtk.org/download/windows.php)), and [Glade](https://glade.gnome.org/) installed.
Glade is Rapid Application Development (RAD) tool used to design user interfacesa graphic interfaces. With Glade we can generate GUI layout easily with event listeners defined and then later programmed in Python to apply modifications according to needs.

Development can be done on Windows, Linux and MacOS. The easiest to work with is Linux as it is more manageable when it comes to adding dependencies. The easiest way to have all the required setup done is to follow these instructions:

#### Linux
1. open up your terminal
2. use your package manager to install the dependency needed to run a demo of the GUI with `python ArmGUILayout.py`:
- `sudo apt-get install python-gtk2`
- `sudo apt-get install libgtk-3-dev`
3. Optional development tools:
- `sudo apt-get install glade` - graphic interface designer
- `sudo apt-get install geany` - text editor
    
Glade setup steps are [here](https://prognotes.net/2015/12/installing-gtk-3-and-glade-development-tools-in-linux/) 

To install the dependencies for the arm position panel:
`sudo apt-get install python-gi-cairo`
`sudo pip install matplotlib`        

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
    - pygobject [docs](http://pygobject.readthedocs.io/en/latest/getting_started.html)  
    - the fix [link](https://github.com/jeffreywildman/homebrew-virt-manager/issues/73)  
    -   Big thanks to [@bnduwi](https://github.com/bnduwi) for taking the time to show me how to get it to work.

### Windows
For Windows there are two options. The installation can be done using a manual install that involves seting up Python and then using an all in one installer to add PygObject which has the GTK+ features. The manual option is more flexible when dealing with dependcies. The other option is to use [MYSYS2](http://www.msys2.org/). MYSYS2 is ok but can present some issues when trying to add dependencies needed during developement. Therefore the manual install option is the best choice.
- For the manual install follow the steps found [here](https://docs.google.com/document/d/19XExylHDHJGtTslYtsATP1ufoPLrfBZetDk1i3PyEb4/edit)
- For the setup using MYSYS2 follow the steps found [here](https://pygobject.readthedocs.io/en/latest/getting_started.html#windows-logo-windows)

For the manual install involving first installing Python then PyGobject seperatly the steps are found [here](https://docs.google.com/document/d/19XExylHDHJGtTslYtsATP1ufoPLrfBZetDk1i3PyEb4/edit)
 
For installation using MYSYS2 the steps are:
1. Download MYSYS2 [here](http://www.msys2.org/). Run the installer.
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
