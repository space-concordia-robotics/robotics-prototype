// 12 Mar 2014
// this works with ComArduino.py and ComArduinoA4e.rb
// this version uses a start marker 254 and an end marker of 255
// it uses 253 as a special byte to be able to reproduce 253, 254 and 255
// it also sends data to the PC using the same system
// if the number of bytes is 0 the PC will assume a debug string and just print it to the screen

//================

#define startMarker 254
#define endMarker 255
#define specialByte 253
#define maxMessage 16

// the program could be rewritten to use local variables instead of some of these globals
// however globals make the code simpler and simplify memory management

byte bytesRecvd = 0;
byte dataSentNum = 0; // the transmitted value of the number of bytes in the package i.e. the 2nd byte received
byte dataRecvCount = 0;


byte dataRecvd[maxMessage];
byte dataSend[maxMessage];
byte tempBuffer[maxMessage];

byte dataSendCount = 0; // the number of 'real' bytes to be sent to the PC
byte dataTotalSend = 0; // the number of bytes to send to PC taking account of encoded bytes

boolean inProgress = false;
boolean startFound = false;
boolean allReceived = false;

//================


/*
- Receives data into tempBuffer[]
- saves the number of bytes that the PC said it sent - which will be in tempBuffer[1]
- uses decodeHighBytes() to copy data from tempBuffer to dataRecvd[]
- the Arduino program will use the data it finds in dataRecvd[]
*/
void getSerialData() {
  // send data only when you receive data
  if (Serial.available() > 0) {
    // read the incoming byte
    byte x = Serial.read();

    if (x == startMarker) {
      bytesRecvd = 0;
      inProgress = true;
      // blinkLED(2);
      debugToPC("start received");
    }

    if (inProgress) {
      tempBuffer[bytesRecvd] = x;
      bytesRecvd ++;
    }

    if (x == endMarker) {
      inProgress = false;
      allReceived = true;

      // save the number of bytes that were sent
      dataSentNum = tempBuffer[1];

      decodeHighBytes();
    }
  }
}

//============================
// processes the data that is in dataRecvd[]
void processData() {

  if (allReceived) {

      // for demonstration just copy dataRecvd to dataSend
    dataSendCount = dataRecvCount;
    for (byte n = 0; n < dataRecvCount; n++) {
       dataSend[n] = dataRecvd[n];
    }

    dataToPC();

    delay(100);
    allReceived = false;
  }
}

//============================
/* 
copies to dataRecvd[] only the data bytes i.e. excluding the marker bytes and the count byte
and converts any bytes of 253 etc into the intended numbers
Note that bytesRecvd is the total of all the bytes including the markers
*/
void decodeHighBytes() {

  dataRecvCount = 0;
  for (byte n = 2; n < bytesRecvd - 1 ; n++) { // 2 skips the start marker and the count byte, -1 omits the end marker
    byte x = tempBuffer[n];
    if (x == specialByte) {
       
debugToPC("FoundSpecialByte");
       n++;
       x = x + tempBuffer[n];
    }
    dataRecvd[dataRecvCount] = x;
    dataRecvCount ++;
  }
}

//====================

void dataToPC() {

      // expects to find data in dataSend[]
      //   uses encodeHighBytes() to copy data to tempBuffer
      //   sends data to PC from tempBuffer
    encodeHighBytes();

    Serial.write(startMarker);
    Serial.write(dataSendCount);
    Serial.write(tempBuffer, dataTotalSend);
    Serial.write(endMarker);
}

//============================
// Copies to temBuffer[] all of the data in dataSend[]
// and converts any bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2 as appropriate
void encodeHighBytes() {

  dataTotalSend = 0;
  for (byte n = 0; n < dataSendCount; n++) {
    if (dataSend[n] >= specialByte) {
      // @TODO run test to see what's the special byte here
      debugToPC("specialByte = ");
      debugToPC(specialByte);
      tempBuffer[dataTotalSend] = specialByte;
      dataTotalSend++;
      tempBuffer[dataTotalSend] = dataSend[n] - specialByte;
    }
    else {
      tempBuffer[dataTotalSend] = dataSend[n];
    }
    dataTotalSend++;
  }
}

//=========================

void debugToPC(char arr[]) {
    byte nb = 0; // number of bytes
    Serial.write(startMarker);
    Serial.write(nb);
    Serial.print(arr);
    Serial.write(endMarker);
}

//=========================

void debugToPC(byte num) {
    byte nb = 0;
    Serial.write(startMarker);
    Serial.write(nb);
    Serial.print(num);
    Serial.write(endMarker);
}

//=========================

void blinkLED(byte numBlinks) {
    for (byte n = 0; n < numBlinks; n ++) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
}

//================

void setup() {
  //pinMode(13, OUTPUT); // the onboard LED
  Serial.begin(57600); // opens serial port, sets data rate to 57600 bps
  debugToPC("Arduino Ready from ArduinoPC.ino");

  delay(500);
  //blinkLED(5); // just so we know it's alive
}

//================

void loop() {

  getSerialData();

  processData();

}
