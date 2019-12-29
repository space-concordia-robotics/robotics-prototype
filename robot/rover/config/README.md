# config

General configuration files to be used on the odroid. Contains files required for setting up gpio pins on the odroid XU4, as well as some udev rules and a sample .bash_aliases file.

## rc.local

### Purpose

The version in this repo exports the mux selector GPIO pins and sets them to output mode.

### Setup
This file already exists by default in the `/etc` folder.
Either overwrite the one you already have or merge the contents of this version with your own to activate gpio pins 21 and 18.


## udev rules

### Workflow
The best way to go about updating udev rules is that you make changes in the repo folder and then `sudo cp` the file to the `rules.d` folder on the OBC so you can both test changes and keep track of changes easily.
When you have verified your changes are good you can copy it over to the repo on your basestation with (as an example for `my-webcams.rules`):

- `scp odroid@odroid:~/Programming/robotics-prototype/robot/rover/config/my-webcams.rules $ROVER/config/`

### 99-com.rules

#### Purpose

On bootup this file will make sure that the group `gpio` will be added to the necessary folders so that any user belonging to that group will be able
to utilize the gpio bash functionality without needing to be root. Those necessary folders are `/sys/class/gpio` and `/etc/devices`.

#### Setup
If this file does not exist in the `/etc/udev/rules.d` folder, create it. Otherwise append the contents to your already existing version.
Then you must make sure to add the default user `odroid` to the group `gpio` with: `usermod -a -G gpio odroid`. You may need to create the group first with:
`sudo groupadd gpio`. You will have to reboot for the effects to take place.

### my-webcams.rules

#### Purpose
On bootup this file will ensure that specific physical usb ports get mapped to specific names using [symlinks](https://en.wikipedia.org/wiki/Symbolic_link) so that we can always refer to them from code accurately, assuming the devices are plugged into the appropriate usb ports.
This was mainly done to handle multiple cameras (of the same make/model, thus virtually indistinguishable) so that they can be programmatically addressed.

#### Setup
This file should be inside the `/etc/udev/rules.d/` folder.

