#!/bin/bash

sudo service snapd restart
sudo snap install yq --channel=v4/stable

echo "Downloading latest release of Arduino 2.0 IDE..."
cd /home/$USER/
curl -s https://api.github.com/repos/arduino/arduino-ide/releases/latest | grep "browser_download_url.*Linux_64bit.zip" | cut -d : -f 2,3 | tr -d \" | wget -i -

echo "Unzipping..."
unzip -qq *Linux_64bit.zip 
mv arduino-ide_*_Linux_64bit/ arduino-ide

./arduino-ide/arduino-ide &

sleep 6
	
pkill arduino-ide

cat ~/.arduinoIDE/arduino-cli.yaml | yq '.board_manager.additional_urls=["https://www.pjrc.com/teensy/package_teensy_index.json"]' | tee ~/.arduinoIDE/arduino-cli.tmp
mv ~/.arduinoIDE/arduino-cli.tmp ~/.arduinoIDE/arduino-cli.yaml

echo "Cleaning up..."
rm arduino-ide_*_Linux*

echo "Done installing Arduino IDE"

