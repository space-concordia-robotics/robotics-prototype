source ~/.bashrc

# Set REPO variable
if [ -z ${GITHUB_WORKSPACE} ]; then
	REPO=/home/$USER/Programming/robotics-prototype
else
	REPO=$GITHUB_WORKSPACE
fi

cd $REPO/robot/rover
# Check if build directory already exists
if [ ! -d $REPO/robot/rover/build ]; then
	echo "Creating build directory"
	mkdir build
fi
cd build
cmake ..
make
