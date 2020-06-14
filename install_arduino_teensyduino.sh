#/usr/bin/env bash

# NOTE: if you stop the process in the middle of the downloads, make sure to delete the newly downloaded files before re-running this script

ARDUINO_IDE_VERSION="1.8.12"
TEENSYDUINO_VERSION="152"
ARDUINO_IDE_FOLDER="$HOME/arduino-$ARDUINO_IDE_VERSION"

# check if arduino path set, will
if [ -z ${ARDUINO_PATH+x} ]; then
    echo "Environment variable ARDUINO_PATH is unset"
    echo "Setting to $ARDUINO_IDE_FOLDER"
    echo "export ARDUINO_PATH="$ARDUINO_IDE_FOLDER"" >> ~/.bashrc
    ARDUINO_PATH="$HOME/arduino-$ARDUINO_IDE_VERSION"
else
    echo "ARDUINO_PATH is already set to '$ARDUINO_PATH'"
    
    # check if safe to proceed with arduino IDE installation
    if [ "$ARDUINO_PATH" == "$ARDUINO_IDE_FOLDER" ] && [ ! -z "$(ls -A $ARDUINO_PATH)" ]; then
        echo "Skipping arduino IDE installation step"
    else
        echo "Installing arduino IDE"
        mkdir -p ~/Arduino/libraries &&
        wget https://downloads.arduino.cc/arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz &&
        tar xf arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz -C $HOME &&
        bash "$HOME/arduino-$ARDUINO_IDE_VERSION/install.sh"
    fi
fi

echo "Installing Teensyduino"
curl -fSL https://www.pjrc.com/teensy/td_$TEENSYDUINO_VERSION/TeensyduinoInstall.linux64 -o TeensyduinoInstall.linux64 &&
chmod +x TeensyduinoInstall.linux64 &&
./TeensyduinoInstall.linux64 --dir=$ARDUINO_PATH

# clean up
rm arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz
rm TeensyduinoInstall.linux64
