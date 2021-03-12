cd /D %~dp0
C:\Qt\Tools\mingw530_32\bin\gcc -c -Wall -DNO_TIMER_SUPPORT -I"..\..\Common\include" -I"hidapi-master/hidapi" -I"..\..\..\DLP_Spectrum_Library\src" Serial.c API.cpp usb.cpp
C:\Qt\Tools\mingw530_32\bin\gcc -shared -o nano_api.dll Serial.o API.o usb.o -Lhidapi-master/windows/debug/ -lhidapi -L..\..\..\DLP_Spectrum_Library\src -llibdlpspec
del *.o
