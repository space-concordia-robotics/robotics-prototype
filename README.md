# robotics-prototype
This repo contains the beginning of the new (2017-2018) Robotics software team code.

## Contributing and Development Environment Instructions
Firstly, this project is built in Python 3.3+. You need to have a version of Python installed that is 3.3+. Make sure that whenever you use `python` or `python3` or whatever later on meets this requirement.

Secondly, it is imperative you use a virtual env (instead of your system Python) to use/contribute to the project. 

### Setup [virtualenv](https://docs.python.org/3.6/library/venv.html#module-venvhttps://virtualenv.pypa.io/en/stable/userguide/)
Navigate to the projects root directory (`cd ~/.../robotics-prototype`) and create the virtual environment):
```
$ virtualenv -p <path/to/python3.x> venv
```
Make sure you supply a path to a Python 3.x binary. In my case, I just supplied it like this:
```
$ virtualenv -p `which python3` venv
```
**Note**: Your actual alias for calling Python might be different (e.g. `python`, `py`, `py3`, etc). You can ensure whichever it might be is version 3+ by typing `which python` in Linux/Mac (bash/zsh) or `where py` in Windows (cmd.exe).

You should see a new directory named `venv` inside the root directory. We can now activate this virtual environment:
Linux/Mac (bash/zsh):
```
$ source venv/bin/activate
```
Windows (cmd.exe):
```
C:\> venv\Scripts\activate.bat
```

You should see a `(venv)` appear at the beginning of your terminal prompt (in Linux and Mac at least) indicating that you are working inside the virtualenv. Now when you install something:
```
(venv) $ pip install <package>
```
It will get installed in the `venv`, and not conflict with other projects installed system-wide.

To leave the virtual environment run:
Linux/Mac (bash/zsh):
```
(venv) $ deactivate
```
Windows (cmd.exe):
```
C:\> deactivate.bat
```

### Install [dependencies](https://pip.pypa.io/en/stable/user_guide/#requirements-files)
First, ensure you activated your `venv` and you are in the root project directory. Then, install the required packages:
```
(venv) $ pip install -r requirements.txt
```
**Note**: If you get any issues with installing the `pygobject` module (which is needed for running the GTK GUI, which will likely be replaced by a Flask front-end soon), read the follwing section on `pygobject3`, otherwise if all was good then just skip over it.

#### Install and configure `pygobject3`
If you ran into issues installing `pygobject` when trying to `pip install -r requirements.txt`, then here are possible reasons:

**(A)** You might not have the pyobject installed on your system, in which case you need to install it via your package manager. For MacOS this was as simple as: 
```
(venv) $ brew install pygobject3 gtk+3
```

**(B)** If you get something along the lines of not being able to find `libffi`, you might need to install it via your package manager. For MacOS again: 
```
(venv) $ brew install libffi
```

Confirm its whereabouts with `locate libffi.pc` (you might be prompted  to built the locate database for which the command for doing so is included in the output message). The output you see should be something along the lines of `/usr/local/Cellar/libffi/3.2.1/lib/pkgconfig/libffi.pc` when you type `locate libffi.pc`. For the virtual Python environment to locate it, simply export the environment variable `PKG_CONFIG_PATH` with the directory where `libffi.pc` is located in:
```
export PKG_CONFIG_PATH=/usr/local/Cellar/libffi/3.2.1/lib/pkgconfig
```
Now you should be able to retry installing the requirements:
```
(venv) $ pip install -r requirements.txt
```
Which should produce no errors!

### Setup [setuptools](https://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode)
When you try to run `pytest` from anywhere inside the project, there's a very good chance you'll get `ModuleNotFoundError` thrown. This is because absolute imports inside each test file won't work if the test is being executed by `pytest` inside the test's own directory (i.e. the `sys.path` variable will only contain the directory where the test file lives, not the root directory). To encourage best practices, and avoid doing dirty/hacky `sys.path` manipulation within each python file, using `setuptools`'s **develop** feature will allow the virtual environment to create temporary dev package installs using "hooks" (eggs) to each path inside package directory as defined by the "name" attribute in `setup.py` (i.e. `robot` module will be available as though it was installed through `pip`). 

In other words, this means all imports of modules inside the `robot` directory should be imported with absolute path, e.g. inside `"tests/unit/motor_test.py"`, the `Motor` class can be imported using:
```
# tests/unit/motor_test.py
from robot.basestation.Motor import Motor
...
```

First we have to install `setuptools`, but our `virtualenv` installs this by default when setting up the environment, so no need!

Still in root project directory (which contains `setup.py`):
```
(venv) $ pip install -e .
```

You should now be able to execute tests without `ModuleNotFoundError` thrown:
```
(venv) $ pytest
```

To remove the package you just installed using:
```
(venv) $ pip uninstall robot
```

**DISCLAIMER:** This issue with module imports via `pytest` was the motivating factor to change the project directory structure. For this technique to work, the 'source' code must live inside (nested) a main directory (usually named the same as project directory name or other suitable representative identifier such as **robot** in this case). The `src` subdirectory was renamed because it made no sense when importing a package module by name like `import src.basestation.Motor`, which has no meaning/place in a module semantic context (`import robot.basestation.Motor` is much more appropriate). Most Python projects do not use a `src` directory unless it's for storing their source code that eventually gets compiled to binary (i.e. such as `.c`, `.h`, etc.. files). Also, `base-station` was renamed to `basestation` because Python no-likey dashes in import statements.

## ODROID
### How to upload Arduino scripts from the odroid

1. Make sure the arduino is plugged into the odroid

2. Copy your Arduino source(s) into platformio/src/

3. Navigate to platformio/ folder

4. Upload the script via the following command: `platformio run -t upload`. This will both compile and upload the code.

Note: I didn't look into adding libraries yet but I'm pretty sure you want to place them in the platformio/lib folder. See [platformio lib help page](http://docs.platformio.org/en/latest/userguide/lib/index.html) to learn more

### IP emailer service

The odroid will send an email with its local IP address every time it boots

This was accomplished by running `syncEmailer.sh` and adding the following line to the crontab via `crontab -e`:

```
@reboot /home/odroid/emailer/runEmailer.sh
```

Let Peter or David know if you want to be added to this mailing list.

### Remote connect from home

- Open a terminal (I recommend git bash if you're using windows)
- SSH into Concordia's network with your netname (type the following into the terminal): 
```
ssh net_name@login.encs.concordia.ca
```
- It should ask you for a password, which will your ENCS password
- Grab the latest IP address of the odroid from your email, then ssh into it: 
```
ssh odroid@ip_address
```
- It should ask you for a password, which will be `odroid`

