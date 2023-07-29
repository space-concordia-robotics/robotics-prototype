# Set REPO variable
if [ -z ${GITHUB_WORKSPACE} ]; then
	REPO=/home/$USER/Programming/robotics-prototype
else
	REPO=$GITHUB_WORKSPACE
fi

cd $REPO/robot/rospackages
# Check if build directory already exists
if [ ! -d ~/Programming/robotics-prototype/robot/rover/build ]; then
	echo "creating build directory"
	mkdir build
fi
cd build
cmake ..
make
