#/usr/bin/env bash

# NOTE: if you stop the process in the middle of the downloads, make sure to delete the newly downloaded files before re-running this script

export ARDUINO_IDE_VERSION="1.8.12"
export ARDUINO_PATH="$HOME/arduino-$ARDUINO_IDE_VERSION"
export TEENSYDUINO_VERSION="152"
mkdir -p ~/Arduino/libraries &&
cd "$HOME/Downloads" &&
wget https://downloads.arduino.cc/arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz &&
tar xf arduino-$ARDUINO_IDE_VERSION-linux64.tar.xz -C /home/$USER/ &&
bash "$HOME/arduino-$ARDUINO_IDE_VERSION/install.sh"
curl -fSL https://www.pjrc.com/teensy/td_$TEENSYDUINO_VERSION/TeensyduinoInstall.linux64 -o TeensyduinoInstall.linux64 &&
chmod +x TeensyduinoInstall.linux64 &&
./TeensyduinoInstall.linux64 --dir=$ARDUINO_PATH &&
cd -
