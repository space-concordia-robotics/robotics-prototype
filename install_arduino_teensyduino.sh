#/usr/bin/env bash

# NOTE: Running this script in sudo will cause the checks for environment vars to fail
# To complete the installation run `sudo install.sh` in the arduino IDE folder

# make sure dependencies are installed
sudo apt install curl

ARDUINO_IDE_VERSION="1.8.12"
TEENSYDUINO_VERSION="152"
ARDUINO_IDE_FOLDER="$HOME/arduino-$ARDUINO_IDE_VERSION"
ARDUINO_DOWNLOAD_URL="https://downloads.arduino.cc/arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz"
TEENSYDUINO_DOWNLOAD_URL="https://www.pjrc.com/teensy/td_$TEENSYDUINO_VERSION/TeensyduinoInstall.linux64"

leftover_download_count=`ls -1 ./arduino-$ARDUINO_IDE_VERSION-* 2>/dev/null | wc -l`   

if [ $leftover_download_count != 0 ]; then
    echo "Deleting leftovers..."
    rm "arduino-$ARDUINO_IDE_VERSION-"*
fi

# check if arduino path set
if [ -z ${ARDUINO_PATH} ]; then
    echo "Environment variable ARDUINO_PATH is unset"
    echo "Setting to $ARDUINO_IDE_FOLDER"
    echo "export ARDUINO_PATH="$ARDUINO_IDE_FOLDER"" >> ~/.bashrc
    ARDUINO_PATH="$ARDUINO_IDE_FOLDER"
else
    echo "ARDUINO_PATH is already set to '$ARDUINO_PATH'"
fi

folder_file_count=`ls -1 $ARDUINO_IDE_FOLDER/arduino* 2>/dev/null | wc -l`   
 
# check if arduino IDE already installed in target folder
if [ "$ARDUINO_PATH" == "$ARDUINO_IDE_FOLDER" ] && [ $folder_file_count != 0 ]; then
    echo "Skipping arduino IDE installation step"
else
    echo "Installing arduino IDE"
    mkdir -p ~/Arduino/libraries &&
    wget $ARDUINO_DOWNLOAD_URL &&
    tar xf arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz -C $HOME &&
        
    # the following line requires sudo rights to complete the symlink
    # but everything else will work
    bash "$HOME/arduino-$ARDUINO_IDE_VERSION/install.sh"
        
    # clean up
    rm arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz
fi

echo "Installing Teensyduino"
curl -fSL $TEENSYDUINO_DOWNLOAD_URL -o TeensyduinoInstall.linux64 &&
chmod +x TeensyduinoInstall.linux64 &&
./TeensyduinoInstall.linux64 --dir=$ARDUINO_PATH

# clean up
rm TeensyduinoInstall.linux64
