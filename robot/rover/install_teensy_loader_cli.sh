#!/bin/bash


if [ -f "/etc/udev/rules.d/49-teensy.rules" ]; then
	echo "teensy rules already installed"
else
	echo "Downloading teensy rules..."
	sudo wget -P /etc/udev/rules.d/ "https://www.pjrc.com/teensy/49-teensy.rules" 
fi 

	sudo apt install --no-upgrade libusb-dev

if [ -f "${ARDUINO_PATH}"/hardware/tools/teensy_loader_cli ]; then
	echo "Teensy Loader CLI already installed"
else
	echo "Installing Teensy Loader CLI..." 
	curl https://github.com/PaulStoffregen/teensy_loader_cli/archive/master.zip -L -o teensy_loader_cli.zip
	unzip -q teensy_loader_cli.zip 
	make -s -C teensy_loader_cli-master
	mv teensy_loader_cli-master/teensy_loader_cli /home/nikolas/arduino-1.8.12/hardware/tools
	rm -rf teensy_loader_cli.zip teensy_loader_cli-master
fi 



