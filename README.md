[![Build Status](https://travis-ci.org/space-concordia-robotics/robotics-prototype.svg?branch=master)](https://travis-ci.org/space-concordia-robotics/robotics-prototype)

# robotics-prototype
This repo contains the Robotics software team code.

## Contributing and Development Environment Instructions

Firstly, this project is built in Python 3.6+ and JavaScript (ES6). You need to have a version of Python installed that is 3.6+. Make sure that whenever you use `python`, `python3` or `python3.6` or whatever later on meets this requirement.

Secondly, it is imperative you use a virtual env (instead of your system Python) to use/contribute to the project, else things could get messy.
### Prerequisites
Make sure you are using Ubuntu 16.04 (The ROS distribution we use doesn't support anything newer than 16.04).

Make sure the following are installed
```
$ sudo apt install python-pip
$ sudo apt install virtualenv
$ sudo apt install git
```
You will also need python3.6, which is by default not in Ubuntu 16.04's APT's available packages.
#### Installing python3.6
```
$ sudo add-apt-repository ppa:deadsnakes/ppa
$ sudo apt update
$ sudo apt install python3.6
```
### Setup the local repository

```
$ cd ~/
$ mkdir Programming
$ cd Programming
$ git clone --recursive https://github.com/space-concordia-robotics/robotics-prototype robotics-prototype
```
A local repository should now be created. `robotics-prototype` is the root directory for this project.

If you do not have access to the GitLab repository, you will not be able to successfully authenticate to clone `rover2018-elec`. This is fine as long as you do not need the PDS code.

If you have access to the GitLab repository, you will need additional setups for GitLab pulling and pushing. Please see [how to create and add your SSH key](https://docs.gitlab.com/ee/gitlab-basics/create-your-ssh-keys.html) and [this](https://stackoverflow.com/questions/47860772/gitlab-remote-http-basic-access-denied-and-fatal-authentication/51133684#51133684) issue you may encounter

### Setup [virtualenv](https://docs.python.org/3.6/library/venv.html#modulevenvhttps://virtualenv.pypa.io/en/stable/userguide/)
```
$ cd robotics-prototype
$ virtualenv -p `which python3.6` venv
$ source venv/bin/activate
```
You should see a `(venv)` appear at the beginning of your terminal prompt (in Linux and Mac at least) indicating that you are working inside the virtualenv. Now when you install something:
```
(venv) $ pip install <package>
```
It will get installed in the `venv`, and not conflict with other projects installed system-wide.

### Install [dependencies](https://pip.pypa.io/en/stable/user_guide/#requirements-files)
```
(venv) $ pip install -r requirements.txt -r requirements-dev.txt
```
### Setup [setuptools](https://setuptools.readthedocs.io/en/latest/setuptools.html#development-mode)
Still in the root directory,
```
(venv) $ python setup.py develop
(venv) $ pytest
```
Running `pytest` without doing `python setup.py develop` will give a ModuleNotFound error. To read up more on this, click [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Troubleshooting)

To deactivate virtualenv, run `deactivate`.
### Install [ROS-Kinetic](http://wiki.ros.org/kinetic)
```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
chmod 755 ./install_ros_kinetic.sh
bash ./install_ros_kinetic.sh
```
To see exactly what happened during the installation of ROS-Kinetic, you can read the script file located in which ever directory it was downloaded in. Your `~/.bashrc` file was modified, and so to make use of the new changes, you should restart your terminal.

To verify ROS-Kinetic has been successfully installed, you should do
```
$ roscore
```
To stop a running process in the command line, press `Ctrl-C`
### Install [rosbridge-suite](http://wiki.ros.org/rosbridge_suite)
```
$ sudo apt install ros-kinetic-rosbridge-suite
```
To verify that its working, deactivate `venv` with `deactivate`
```
$ roslaunch rosbridge_server rosbridge_websocket.launch
```
You will need `venv` activated for everything except the above command until [this issue](https://github.com/space-concordia-robotics/robotics-prototype/issues/197) is resolved

### Setup [catkin workspaces](http://wiki.ros.org/catkin/conceptual_overview)
The first catkin workspace was automatically generated during the scripted installation of ROS, you can see `catkin_ws` in your `~` directory.

You need to setup another catkin workspace in robot/rospackages
```
$ cd ~/Programming/robotics-prototype/robot/rospackages
$ catkin_make
```
### .bashrc edits
You should add this to your `~/.bashrc` file. To automatically open `~/.bashrc` using the GNU nano text editor, you can run `eb`. (this shortcut was added in your .bashrc file during the scripted ROS installation, among a few others)
```
#competition mode
#export ROS_MASTER_URI=http://172.16.1.30:11311
#export ROS_HOSTNAME=$USER

. ~/Programming/robotics-prototype/robot/rospackages/devel/setup.bash
. ~/Programming/robotics-prototype/venv/bin/activate
source ~/Programming/robotics-prototype/robot/basestation/config/.bash_aliases
```
**Open a new terminal for changes to apply**. You should automatically have a virtual environment activated. The last line added a couple of aliases. You can read the .bash_aliases file to see all the new aliases.

### Setup env.js
You will need to provide the IP that the GUI will work with. By running this script, everything is handled.

```
./robot/basestation/env.sh > robot/basestation/static/js/env.js
```

### Run the GUI
Firstly, in a new terminal you should run `rosgui` (this is one of the new aliases in .bash_aliases that you added) to launch a ROS server

Back in the original terminal, you can make use of the `base` alias, to be automatically directed to the basestation directory, and then run the app.py file.

```
$ base
$ python app.py
```
Alternatively, after running `rosgui` you can run `startgui` to run the GUI. If you run `startgui` it will also run the `updateEnv` alias which makes sure that `env.js` is setup.
### ffmpeg

For the feature of capturing a snapshot of our camera stream we need `ffmpeg`:
```
sudo apt install ffmpeg
```

You can read about the formatting guide [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Code-Formatting-and-Conventions)

## Using Git
For a quick primer on our workflow using git, [CLICK HERE :)](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions)

### Git hooks

This explains how to setup git hooks which prepend an issue number to a commit message, and verify branch naming conventions. Git hooks are located in the local `.git/hooks` file of each repo, so initializing this hook in the this repo will not change any other repos you might have.

### Setting up git hooks

Git hooks are important for performing repository validity checks. To setup git hooks using Git Bash, run the following commands from the root of the repository (ex: from ~/Programming/robotics-prototype/):

- `cp commit-message-hook.sh .git/hooks/prepare-commit-msg`
- `cp branch-verification-hook.sh .git/hooks/post-checkout`

If you're on windows, install [Git Bash](https://git-scm.com/downloads) to be able to run the same commands.

### The commit hook prepender and branch verification hook

Now, when ever you using `git commit -m` commit-message-hook will prepend the issue number to your message.
This will show up in the repo as [#\<issue number\>], so there is no longer a need to add this to your commit message.
This will work as long as your branches are named using our branch naming standards defined in our wiki, otherwise the commit will be aborted. For more information on our conventions check [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions).

In order to write a long commit message using `git commit -m`, write a concise title and then press enter twice.
Then, type as long a message as is appropriate and close the quotation mark. This ensures it will be formatted nicely on github.

Finish the commit and `git push` as usual.

Lastly, the branch-verification-hook will verify if the names of newly created branches follow our naming conventions explained [here](https://github.com/space-concordia-robotics/robotics-prototype/wiki/Git-Workflow-and-Conventions). Note that this hook will only run after entering git checkout <branch-name>, and not when the branch is created.

### Cloning and Pulling
We are using git submodules in `robotics-prototype`. This means that we are using code that is external to our repository. To ensure that it also downloads all the packages from the external repository, use the commands below :

Clone : `git clone --recursive https://github.com/space-concordia-robotics/robotics-prototype`

Pull : `git pull; git submodule update --init --recursive`

If you do not have access to the GitLab repository, you will not be able to successfully authenticate to clone `rover2018-elec`. This is fine as long as you do not need the PDS code.

If you have access to the GitLab repository, you will need additional setups for GitLab pulling and pushing. Please see [how to create and add your SSH key](https://docs.gitlab.com/ee/gitlab-basics/create-your-ssh-keys.html) and this [issue](https://stackoverflow.com/a/51133684/4048657) you may encounter

### Atom
If you're using Atom (it can be installed via Ubuntu software), setting up should be fairly easy.

- Run `apm install --packages-file .atom/package-list.txt` (from project root). This should install all needed packages.
- Note that the config file `./atom/config.cson` (still in the project root) is where the configurations for said packages are stored/versioned for this project.
