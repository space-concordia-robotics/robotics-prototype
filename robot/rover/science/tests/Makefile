BASE_PROJECT := science
#Make sure that this has the same filename as the .cpp file you are testing, it should be in this directory
APP_NAME := ScienceTests
UNIXHOSTDUINO_PATH := ~/Arduino/libraries/UnixHostDuino

#List of paths where the directories containing the .cpp files are located
SOURCE_DIRS_PATH := ../../
#Every directory within the paths will be searched for the .cpp files
SOURCE_DIRS := AUnit $(BASE_PROJECT)

include $(UNIXHOSTDUINO_PATH)/UnixHostDuino.mk

$(APP_NAME).exe : $(APP_NAME).out
	./$(APP_NAME).out