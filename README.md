[![Build Status](https://travis-ci.org/space-concordia-robotics/robotics-prototype.svg?branch=master)](https://travis-ci.org/space-concordia-robotics/robotics-prototype)

# robotics-prototype
This repo contains the Robotics software team code. For a quick primer on our workflow using git, [CLICK HERE :)](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions)

## Contributing and Development Environment Instructions

Firstly, this project is built in Python 3.6+ and JavaScript (ES6). You need to have a version of Python installed that is 3.6+. Make sure that whenever you use `python`, `python3` or `python3.6` or whatever later on meets this requirement.

Secondly, it is imperative you use a virtual env (instead of your system Python) to use/contribute to the project, else things could get messy.

### Setup [virtualenv](https://docs.python.org/3.6/library/venv.html#modulevenvhttps://virtualenv.pypa.io/en/stable/userguide/)
Navigate to the projects root directory (`cd ~/.../robotics-prototype`) and create the virtual environment):
Make sure you supply a path to a Python 3.6.x+ binary. Make sure to use backticks (not single quotes) for `which python3.6`:
```
$ virtualenv -p `which python3.6` venv
```
**Note**: Your actual alias for calling Python might be different (e.g. `python`, `py`, `py3`, etc). You can ensure whichever it might be is version 3.6+ by typing `which python` in Linux/Mac (bash/zsh).

You should see a new directory named `venv` inside the root directory. We can now activate this virtual environment:
Linux/Mac (bash/zsh):
```
$ source venv/bin/activate
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

### Install [dependencies](https://pip.pypa.io/en/stable/user_guide/#requirements-files)
First, ensure you activated your `venv` and you are in the root project directory. Then, install the required packages:
```
(venv) $ pip install -r requirements.txt -r requirements-dev.txt
```

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
(venv) $ python setup.py develop
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

Finally, to link ROS to our webpage we need to install rosbridge: `sudo apt install ros-kinetic-rosbridge-suite`.
Before running the GUI with `./app.py`, you need to run in a separate terminal: `roslaunch rosbridge_server rosbridge_websocket.launch`. Make sure to deactivate the `venv` for this terminal (otherwise it will crash).
Make sure it works fine: after running the `./app.py` go visit `localhost:5000` in your browser (recommended browser: Google Chrome).

### .bashrc edits
You should edit your `~/.bashrc` file so that it looks like this:

```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# local mode
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_HOSTNAME=localhost
# competition mode
export ROS_MASTER_URI=http://172.16.1.30:11311 # competition mode
export ROS_HOSTNAME=beep
. /home/beep/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. /home/beep/Programming/robotics-prototype/venv/bin/activate
```

Note that the first 4 lines are there by default when you do the ROS full install,
and that you will have to change the paths in the last two lines to reflect where your files are installed on your computer.
Additionally you need to build the packages in the rospackages folder.
`cd` into `rospackages` and run `catkin_make`, otherwise the second last line in the above snippet will fail (as the folder devel will not have yet been generated).

### Commit message hook

This explains how to use and setup a git hook that will prepend the issue number to a commit message.
Git hooks are used to automate git commands and functions.
There are a number of different options, but this particular hook will add the issue number to your
`git commit -m` message once it is pushed to the repository.
Git hooks live in a local `.git/hooks` file of each repo, so initializing this hook in the this repo will not change any other repos you might have.

#### Setting up a git hook

If you're in Linux run the following command. If you're on windows, install [Git Bash](https://git-scm.com/downloads) to be able to run the same command.

From the root of the repository:
- `cp commit-message-hook.sh .git/hooks/prepare-commit-msg`

#### Using the commit hook prepender

Now, when ever you using `git commit -m` the hook will prepend the issue number to your message.
This will show up in the repo as [#\<issue number\>], so there is no longer a need to add this to your commit message.
This will work as long as your branches are named using our branch naming standards defind in our wiki, otherwise the commit will be aborted.

In order to write a long commit message using `git commit -m`, write a concise title and then press enter twice.
Then, type as long a message as is appropriate and close the quotation mark. This ensures it will be formatted nicely on github.

Finish the commit and `git push` as usual. For more information on our conventions check [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Workflow-and-Conventions).

### ffmpeg

For the feature of capturing a snapshot of our camera stream we need `ffmpeg`:
- sudo apt-get install ffmpeg

### Formatting Guide
When you install the `requirements-dev.txt.` dependencies, you will have `pylint` and `yapf` installed. Both of these packages allow for set guidelines on how code should behave (`pylint`) and how it should look (`yapf`). In other words, `pylint` is the project's linter and `yapf` is the auto-formatter. You can read more about these online but the basic principle is that we should all have code that looks alike and behaves properly based on some established set of heuristics. The `.pylint` file (based entirely on Google's very own one) contains the configurations that `pylint` uses to validate the code. If you configure your IDE properly, both the linter (`pylint`) and autoformatter (`yapf`) should work without prompting any action. Here is an example of the project opened in VSCode (which has it's configurations outlined in `.vscode/settings.json`) showing how `pylint` indicates things (also shown clickable `pytest` actions right inside the source!):
![VSCode putting `pylint` and `pytest` to work!](docs/media/pylint-pytest-vscode.png)

You might not be able to tell from the screenshot, but whenever the incorrect indentation or needless linebreaks are entered, `yapf` automatically formats the code to comply with the project's format rules.

All in all, by using these tools, we will ensure that the codebase is consistent is less of a pain to maintain in the long run or for any new entrants.

Although most of the syntax/format will be handled by `pylint`/`yapf`, some things that aren't are briefly outlined here (namely regarding source file naming guidelines):
- As stated [here](https://github.com/google/styleguide/blob/gh-pages/pyguide.md#3163-file-naming-s3163-file-naming), file names shouldn't include dashes since they need to be importable.
- Although class names use `CapWords`, modules should have `lower_with_under.py` names. This is to prevent confusing with imports on whether or not the module itself or the class was imported as described [here](https://github.com/google/styleguide/blob/gh-pages/pyguide.md#3162-naming-convention). This means even if you file contains only one class like `Motor`, the filename (i.e. module name -- each Python file is considered a module) should be `motor.py` and **not** ~~`Motor.py`~~.
- Test files should be named `modulename_test.py` (note the `_test` appearing as a suffix, not prefix) with the class inside named `TestModuleName` (here `Test` needs to be a prefix, blame `pytest` for that). This class should encapsulate the methods that test various functionality or states named `test_<functionality_or_state>(self)` (same for functions). Note that these guidelines will ensure that your tests will be recognized by [`pytest`'s test discovery](https://docs.pytest.org/en/latest/goodpractices.html#test-discovery).

#### Atom
If you're using Atom, setting up should be fairly easy.

- Run `apm install --packages-file .atom/package-list.txt` (from project root). This should install all needed packages.
- Note that the config file `./atom/config.cson` is where the configurations for said packages are stored/versioned for this project.

## ODROID
### Flashing ubuntuMATE to the odroid (tested on Ubuntu 16.04)

Sometimes things go wrong, like when all of a sudden the odroid won't properly boot up anymore.
The simplest fix for this usually is reflashing the OS onto your memory card.
When doing this, you are overwriting all previous data on the memory card so if you want to retain and are able to access your user space data then back it up prior to proceeding.
If you wanted to back up your Documents folder then use `sudo cp -rp /home/odroid/Documents /media/backup/my_home` with the `-rp` flag if you want to keep owners and permissions of the files intact.
For storage media types you are either using a microSD or an eMMC card. The method described below will work for both (so far only tested on eMMC) but you should know that you will need some kind of adapter to be able to access the memory cards as storage devices.

#### microSD
For the microSD card your typicial microSD to SD reader will do. If you don't have a microSD port on your computer you can use a microSD to USB adapter.

#### eMMC
For the case of the eMMC card you will need to use an eMMC reader, along with a microSD to SD or USB adapter.[This forum post](https://forum.odroid.com/viewtopic.php?f=53&t=2725) contains a list of verified eMMC readers, microSD/USB adapters.

In my case I used the following eMMC reader to microSD, paired with a Transcend USB adapter.

1. eMMC module reader

<img src="docs/media/emmc-module-reader.jpg" height="250" width="250">

2. eMMC module mounted onto reader

<img src="docs/media/emmc-reader-mounted.jpg" height="500" width="500">

3. eMMC module reader mounted onto USB adapter microSD slot

<img src="docs/media/emmc-reader-usb-mounted.jpg" height="500" width="500">

4. Plugged in

<img src="docs/media/usb-emmc-live.jpg" height="500" width="500">

The next step is to download the ubuntuMATE 16.04 image from [here](https://odroid.in/ubuntu_16.04lts/). Choose the proper image depending on the type of odroid you have.
For the odroid we are using in competition (xu4) I used `ubuntu-16.04.2-mate-odroid-xu4-20170510.img.xz`.

If it is not already installed, you will need to install `xz-utils` via `sudo apt-get install xz-utils`.

Now decompress the compressed image with: `unxz ubuntu-16.04.2-mate-odroid-xu4-20170510.img.xz` to obtain the img file.

Use the command `lsblk` to identify the block device that is the eMMC/microSD card you are trying to flash.
To be sure, unplug your adapter first and run `lsblk`, then plug in the adapter and run `lsblk` to see the difference.
In my case it was `/dev/sdb`.

Finally, to flash the image onto the card, you can use the `ddrescue` unix library.

To install it run `sudo apt-get install ddrescue`.

To flash it run `sudo ddrescue -D --force ubuntu-16.04.2-mate-odroid-xu4-20170510.img.xz /dev/sdb`, making sure to replace `/dev/sdb` with your own proper value.

By the time it is done, you will have a fresh install of ubuntuMATE on your card.
**Note** that when booting up for the very first time after a flash the odroid will boot to login screen, but then suddenly die.
This is normal,  after the initial bootup you will need to reboot one more time for it to finally work properly.

Note that the first time you connect your odroid to the internet it will most likely automatically start the job `apt-daily.service` which will temporarily make the `apt-get` command unavailable.
To confirm this, you can check with `ps aux | grep apt` to get the process id (pid). This process will lock the use of `apt-get`.
The best thing to do is to just wait until it is done. You can see how long it has been running with `ps -o etime= -p process_id` (replacing "processid" with the corresponding pid of the apt daily update process.
To get a running time of elapsed time updated each second you may run `watch -n 1 ps -o etime= -p process_id`.

### Custom images

These custom images are made by directly setting up the changes on a fresh image, then using `dd` to save the state of the operating system as an image file.

Example: `sudo dd if=/dev/sdb of=ubuntuMateDev.img bs=4M`

Any kind of authentication requiring services (such as a connection to a wifi network requiring username/password) will not be included in the images.
To find all the custom images follow [this link](https://drive.google.com/drive/u/1/folders/17An9xUkBo8dGS8PoY54tsNWucJLyBDJC) to the corresponding google drive folder.

For further details on these images please see the [Custom Odroid Images wiki page](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Custom-Odroid-Images).

### How to upload Arduino scripts from the odroid

1. Make sure the Arduino is plugged into the odroid

2. Copy your Arduino source(s) into platformio/src/

3. Navigate to platformio/ folder

4. Upload the script via the following command: `platformio run -t upload`. This will both compile and upload the code.

Note: I didn't look into adding libraries yet but I'm pretty sure you want to place them in the platformio/lib folder. See [platformio lib help page](http://docs.platformio.org/en/latest/userguide/lib/index.html) to learn more

### Systemd services

[This link](https://gist.github.com/pdp7/d4770a6ba17e666848796bf5cfd0caee) explains how to setup a systemd service in steps.

The systemd service files are versioned under `robot/rover/systemd`.

#### Ethernet config service

The odroid has two ethernet network configurations:

- `WiredConnection1` for when it is being connected to a regular router, dynamic IP
- `RoverOBC` for when it is being connected to the RocketM900 radio, static IP

On bootup, if an ethernet cable is plugged in, the config script will switch to `WiredConnection1` and attemp to ping google server as a test.
If the test fails, the `RoverOBC` connection type is selected.

This was accomplished by running `syncConfigEthernet.sh` and setting up a systemd startup service to run `runEthernetConfig`.

#### IP emailer service

The odroid will send an email with its local IP address after having run the ethernet config service first, and only if it does not end up selecting the `RoverOBC` configuration.

This was accomplished by running `syncEmailer.sh` and setting up a systemd startup service to run `runEmailer`.

Let Peter know if you want to be added to this mailing list.

#### ROS Rover Startup service

This service follows the IP-emailer service and executes a roslaunch file `rover.launch`, which will allow developers to easily add nodes to be launched on startup without having to create
a new systemd service each time. It automatically starts the rosmaster on the odroid.

This was accomplished by setting up a systemd startup service to run `runRosRoverStart.sh`.

### Remote connect from home

**NOTE:** This will only work if the odroid is powered up and connected to the internet at Concordia.

- Open a terminal (git bash is recommended if you're using windows)
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

### Video Streaming Procedure

This will explain the steps necessary to setup the odroid with rocket M900 radios and the GUI to view a video stream from the odroid.

#### Hardware setup:

1. Setup the rocket M900 radios

- Use the rocket labeled `192.168.1.45` for the odroid, and the one labeled `192.168.1.40` for the basestation.
Connect the power adapter to a power socket, use ethernet cables to connect `POE` (power over ethernet) to the `LAN` port on the radio.
Make sure that you connect some omni-directional antennas to `Chain0` or `Chain1` connectors on the radios. Make sure to use the same for both.
DO NOT connect `LAN` to `LAN`, this can cause issues. After having done the last step, connect the `LAN` port from the power adapter to either the odroid or basestation ethernet port.
When both LEDs on the radios are on (green, red, yellow/orange) as opposed to the first two green ones then both radios are connected to each other.
For an example diagram of the connections see the description of [this](https://github.com/space-concordia-robotics/robotics-prototype/pull/80) pull request.
The only difference is that the ip addresses start with `172.16` vs `192.168`, and that if you are using the laptop I donated the IP address should end with `25` rather than `20`.
- Power up the Odroid using it's power adapter. You should see a solid blue LED at first which then will begin to flash (this indicates normal boot behavior).
- Insert the USB connection for the webcam to the odroid (preferably 3.0 USB hub) _after_ it has booted up.
- Unfortunately this step is not automated: When the camera powers up it will show two options.
Select `PC Camera` using the up/down navigation buttons on the side and click OK (on the top) to select it.

2. Run corresponding software

- After logging in to the basesation computer, select the correct ethernet configuration by clicking on the wifi logo button in the top right corner and clicking (if using the donated laptop) `Base station direct 1`. This will properly set the computers IP for competition mode.
- Open a terminal run `base` which will take you to the basestation folder.
- Run `./app.py` which will run the GUI on `localhost:5000`
- Open chrome and go visit the link `localhost:5000`
- Hover over the buttons to see a description of what they should do. Currently `B` is for enable stream and `N` is for disable.
- Enable the stream with `B`, wait for the response to appear in the console log. If in the response you see a succesful message like: "started stream", you should see the video stream appear.

## TEENSY
### Flashing the Teensy from a PC

Flashing code to a Teensy is very similar to flashing to Arduino, but the Teensyduino add-on is required. [This link](https://www.pjrc.com/teensy/td_download.html) explains how to set up the Arduino IDE and Teensyduino on Windows, Linux and Mac. I (Josh) use Arduino 1.8.5 with Teensyduino 1.42 on Windows 10 x64. In the case of Linux, make sure that the installer is executable: `chmod +x TeensyduinoInstall.linux64` or replace it with whatever the installer's name is.

[This link](https://www.pjrc.com/teensy/td_usage.html) explains how to flash the sketch.

Teensy programming has all of the Arduino functions, as well as extra features described on [the pjrc website](https://www.pjrc.com/teensy/td_libs.html). Currently used features are digitalWriteFast(), IntervalTimer and ellapsedMillis.

I (Josh) currently use Sublime Text to write my code, and then compile it using the Arduino IDE and use the Serial Monitor to communicate with the Teensy. The recommended editors are Atom or VS Code as the rest of this project has configuration files for those text editors.

### A note on ROS for Teensy 3.5/3.6

The Rosserial library must be installed to communicate with a ROS network. The Teensy 3.5 and 3.6 in particular give compiler errors upon using the correct rosserial syntax, however, even though it seems to work on Arduino. The following instructions are a first step towards getting it to work correctly, because at the very least they get rid of the compiler errors. Further research into the library's source code is needed to figure out why it doesn't work on Teensy 3.5/3.6.

 For starters, add `|| defined(__MK65FX512__) || defined(__MK66FX1M0__)` to ArduinoIncludes.h to indicate that Teensy 3.5/3.6 are allowed to use the library. On Windows, this file is located in `Documents\Arduino\libraries\Rosserial_Arduino_Library\src` once rosserial has been installed.

In order to fully be able to control whether Serial (usb) or Serial1 to Serial6 (hardware serial) is being used, in the same file, the following two lines must be commented out:
```
#include <usb_serial.h>  // Teensy 3.0 and 3.1
#define SERIAL_CLASS usb_serial_class
```
and the following lines must be added instead:
```
#ifdef USE_TEENSY_HW_SERIAL
  #include <HardwareSerial.h>
  #define SERIAL_CLASS HardwareSerial
#else
  #include <usb_serial.h>
  #define SERIAL_CLASS usb_serial_class
#endif
```
If you want to choose a different hardware Serial port, the following block of code presented [here](https://answers.ros.org/question/198247/how-to-change-the-serial-port-in-the-rosserial-lib-for-the-arduino-side/#post-id-295159) may work, though I have never tested it. There may be another method:
```
class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 57600){};
};

ros::NodeHandle_<NewHardware>  nh;
```
Where Serial1 can be anything from Serial1 to Serial6.
