#!/bin/bash

if [ -f "/etc/udev/rules.d/49-teensy.rules" ]; then
	echo "Permissions file already exists"
else
	echo "Downloading permissions..."
	wget -P /etc/udev/rules.d/ "https://www.pjrc.com/teensy/49-teensy.rules" 
fi 


if  dpkg --get-selections | grep -q "^$pkg[[:space:]]*libusb-dev$" >/dev/null; then
	echo "Installing libsub-dev"
	apt-get install libusb-dev;
else
	echo "Libusb-dev already installed"
fi

if [ -e "${ARDUINO_PATH}"/hardware/tools/teensy_loader_cli ]; then
	echo "Teensy loader CLI already installed"
else
	echo "Installing Teensy Loader CLI" 
	curl https://github.com/PaulStoffregen/teensy_loader_cli/archive/master.zip -L -o teensy_loader_cli.zip
	unzip -q teensy_loader_cli.zip 
	make -s -C teensy_loader_cli-master
	mv teensy_loader_cli-master/teensy_loader_cli "${ARDUINO_PATH}"/hardware/tools
	rm -rf teensy_loader_cli.zip teensy_loader_cli-master

fi 



