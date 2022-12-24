![ROS CI](https://github.com/space-concordia-robotics/robotics-prototype/workflows/ROS%20CI/badge.svg)

# robotics-prototype
This repo contains the Robotics software team code.

## Contributing and Development Environment

This project uses python virtual environments, so it is necessary to ensure that the virtual environment is properly setup and indeed uses python3.8+.

## Automatic Setup
**Notes:**
- Make sure you have at least 10GB of **free space** to complete the installation. More is always better.
- The script will take about an hour to complete (depends on internet speed and specs). Please monitor the process for any potential errors.
- You should be prompted once (maybe more) for sudo password.
Apart from that, you can sit back and relax :slightly_smiling_face:

To use the automatic setup script you must first clone the repo using the command below. **Make sure that you do not have a directory named `robotics-prototype` in `/home/$USER/Programming` or it will be overwritten.**
```
$ git clone git@github.com:space-concordia-robotics/robotics-prototype.git ~/Programming/robotics-prototype
```
When you have cloned the repo you can then execute `EnvironmentSetup.sh` which will setup the environment. Make sure to uninstall previous ROS installations or the script will exit.
```
$ cd ~/Programming/robotics-prototype
$ ./EnvironmentSetup.sh
```
After you have restarted your terminal you can run the GUI

### Running the GUI TODO: port GUI to ROS2
1. Make sure your virtual environment is activated.
2. In a terminal window run `rosrun controller script`


## Manual setup
If for some reason the automatic script doesn't work, you can follow these steps to set up the development environment.


### Prerequisites
It is recommended to be using Ubuntu 22.04 (although it is possible to use other linux distros, Windows or MacOS if desired).


Install required packages
```
$ sudo apt install python-venv git python-pip
```

Clone the repo.
```
$ cd ~/
$ mkdir Programming
$ cd Programming
$ git clone git@github.com:space-concordia-robotics/robotics-prototype.git
```
A local repository should now be created. `robotics-prototype` is the root directory for this project.

### Setup [virtualenv](https://docs.python.org/3.6/library/venv.html#modulevenvhttps://virtualenv.pypa.io/en/stable/userguide/)
```
$ cd robotics-prototype
$ python -m venv venv
$ source venv/bin/activate
```
You should see a `(venv)` appear at the beginning of your terminal prompt (in Linux and Mac at least) indicating that you are working inside the virtualenv. Now when you install something:
```
(venv) $ pip install <package>
```
It will get installed in the `venv`, and not conflict with other projects installed system-wide.

### Install [dependencies](https://pip.pypa.io/en/stable/user_guide/#requirements-files)
```
(venv) $ pip install -r requirements.txt
```
### Setup [setuptools](https://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode)
Still in the root directory,
```
(venv) $ python setup.py develop
(venv) $ pytest
```
Running `pytest` without doing `python setup.py develop` will give a ModuleNotFound error. To read up more on this, click [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Troubleshooting)

To deactivate virtualenv, run `deactivate`.

### Install and setup [Arduino](https://www.arduino.cc/) + [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)

Run `./install_arduino_teensyduino.sh`

After the script is done, you should be able to run arduino by `cd $HOME/arduino-<version-numver>/` and running: `./arduino`.

To verify that Teensyduino was properly setup, go to `Tools --> Board` and make sure you see options that include "Teensy" in their names. To be extra sure, you can try uploading a sketch to a teensy as well.

### Install [ROS-Galactic](https://docs.ros.org/en/galactic/index.html)
```
bash ./install_ros.sh
```
To see exactly what happened during the installation of ROS-Galactic, you can read the script file located in which ever directory it was downloaded in. Your `~/.bashrc` file was modified, and so to make use of the new changes, **you should restart your terminal**.

To verify ROS-Galactic has been successfully installed, you should do
```
$ ros2
$ printenv ROS_DISTRO
```
In the output of the last command you should see included: `galactic`

To stop a running process in the command line, press <kbd>Ctrl-C</kbd>

### .bashrc edits
You should add this to your `~/.bashrc` file. To automatically open `~/.bashrc` using the GNU nano text editor, you can run `eb`. (this shortcut was added in your .bashrc file during the scripted ROS installation, among a few others)
```
. ~/Programming/robotics-prototype/robot/rospackages/install/local_setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases
```
**Open a new terminal for changes to apply**. You should automatically have a virtual environment activated. The last line added a couple of aliases. You can read the .bash_aliases file to see all the new aliases.

### Setup git hooks

Git hooks are important for performing repository validity checks. To setup git hooks using Git Bash, run the following commands from the root of the repository (ex: from ~/Programming/robotics-prototype/):

- `cp commit_message_hook.py .git/hooks/prepare-commit-msg`
- `cp branch_name_verification_hook.py .git/hooks/post-checkout`

### The commit hook prepender and branch verification hook

Now, when ever you using `git commit -m` commit-message-hook will prepend the issue number to your message.
This will show up in the repo as [#\<issue number\>], so there is no longer a need to add this to your commit message.
This will work as long as your branches are named using our branch naming standards defined in our wiki, otherwise the commit will be aborted. For more information on our conventions check [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions).

In order to write a long commit message using `git commit -m`, write a concise title and then press enter twice.
Then, type as long a message as is appropriate and close the quotation mark. This ensures it will be formatted nicely on github.

Finish the commit and `git push` as usual.

Lastly, the branch-verification-hook will verify if the names of newly created branches follow our naming conventions explained [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions). Note that this hook will only run after entering git checkout <branch-name>, and not when the branch is created.

## Extra Info
- If you want to code in C++ using a Teensy, you will need to read [this wiki page](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Setting-up-Teensyduino).

- You can read about the code formatting guide [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Code-Formatting-and-Conventions)

### Using Git
For a quick primer on our workflow using git, [CLICK HERE :)](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions)

### Cloning and Pulling Updates for Dependencies
We managed our third party dependencies with the cli tool: "vcs import"
- `sudo apt-get install python-vcstools`
\
\
In the robot/rospackages/ directory run:
- `vcs import < in-house-dependencies.repos`
- `vcs import < third-party-dependencies.repos`
