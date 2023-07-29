cd ~/Programming/robotics-prototype/robot/rover
# Check if build directory already exists
if [ ! -d ~/Programming/robotics-prototype/robot/rover/build ]; then
	echo "creating build directory"
	mkdir build
fi
cd build
cmake ..
make
