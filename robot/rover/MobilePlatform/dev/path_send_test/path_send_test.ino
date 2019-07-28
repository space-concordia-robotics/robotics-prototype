/*
Tests the path sending and receiving code.
*/

#include <SoftwareSerial.h>
#include <ArduinoBlue.h>


// The bluetooth tx and rx pins must be supported by software serial.
// Visit https://www.arduino.cc/en/Reference/SoftwareSerial for unsupported pins.
// Bluetooth TX -> Arduino D8
const int BLUETOOTH_TX = 8;
// Bluetooth RX -> Arduino D7
const int BLUETOOTH_RX = 7;

int button;

bool hasPathPrinted = false;

SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth); // pass reference of bluetooth object to ArduinoBlue constructor

// Setup code runs once after program starts.
void setup() {
	// Start serial monitor at 115200 bps.
	Serial.begin(115200);

	// Start bluetooth serial at 9600 bps.
	bluetooth.begin(9600);

	// delay just in case bluetooth module needs time to "get ready".
	delay(100);

	Serial.println("\n\n______________________________________________________________________________________________________________________________________");
	Serial.println("SETUP COMPLETE");
	Serial.println("PATH SEND TEST\n\n");
}

void printCoordinate(float x, float y) {
	Serial.print(x); Serial.print("\t");
	Serial.println(y);
}

// Put your main code here, to run repeatedly:
void loop() {
	button = phone.getButton();
	if (button != -1) {
		// Test if bluetooth is working properly by pressing any button.
		Serial.print("Button: "); Serial.println(button);
	}

	if (phone.isPathAvailable() && !hasPathPrinted) {
		// PRINT INTERPOLATED PATH
		// Get the path data
		int length = phone.getPathLength();
		float * pathX = phone.getPathArrayX();
		float * pathY = phone.getPathArrayY();

		// Print the path data
		Serial.println("______________________________________________________________________________________________________________________________________");
		Serial.println("ACTUAL PATH\n");
		for (int i = 0; i < length; i++) {
			printCoordinate(pathX[i], pathY[i]);
		}

		// PRINT INTERPOLATED PATH
		double step = 3;
		double distance, xCurr, yCurr, x0, y0, x1, y1;
		int numPtsIter;

		Serial.println("______________________________________________________________________________________________________________________________________");
		Serial.println("INTERPOLATED PATH\n");

		for (int i = 1; i < length - 2; i++) {
			// 1. Get the ith and (i+1)st coordinate
			x0 = pathX[i];
			y0 = pathY[i];
			x1 = pathX[i + 1];
			y1 = pathY[i + 1];

			// 2. Get distance between them
			distance = sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0));

			// 3. Get number of points needed
			numPtsIter = (int) distance / step;

			/*Serial.print("x0: "); Serial.print(x0);
			Serial.print("\ty0: "); Serial.print(y0);
			Serial.print("\tx1: "); Serial.print(x1);
			Serial.print("\ty1: "); Serial.print(y1);
			Serial.print("\td: "); Serial.print(distance);
			Serial.print("\tnum: "); Serial.println(numPtsIter);*/

			// 4. Interpolate and print
			xCurr = x0;
			yCurr = y0;
			printCoordinate(x0, y0); // Print the ith coordinate
			for (int j = 0; j < numPtsIter; j++) {
				xCurr += step;
				yCurr = phone.getPathY(xCurr);
				printCoordinate(xCurr, yCurr);
			}
			printCoordinate(x1, y1); // Print the (i+1)st coordinate
		}

		hasPathPrinted = true; // So it won't keep on printing
	}
}
