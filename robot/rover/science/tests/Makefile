#Make sure that this has the same filename as the .cpp file you are testing, it should be in this directory (will later be expanded to support multiple .cpp files)
#This should also be the only the only thing you need to change
APP_NAME := ScienceTests

UNIXHOSTDUINO_PATH := ~/Arduino/libraries/UnixHostDuino

#Location of additional header files needed by the tests
SRC_INCLUDE_DIRS:= ../include

#Addtional .cpp files that are needed by the tests
SRC_FILES := ../src/Carousel.cpp


include $(UNIXHOSTDUINO_PATH)/UnixHostDuino.mk
$(APP_NAME).exe : $(APP_NAME).out
	./$(APP_NAME).out
