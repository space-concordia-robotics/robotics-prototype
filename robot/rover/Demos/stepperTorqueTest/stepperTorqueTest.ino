const int dirpin = 20;
const int steppin = 21;
const int enablepin = 17;

bool isTurning = false;
elapsedMicros stepTimer;
unsigned int stepInterval = 25000;
int accel = 0;

char serialBuffer[100];

void printCommands(void) {
  Serial.println("enable: enable stepper driver");
  Serial.println("disable: enable stepper driver");
  Serial.println("duration: set step duration");
  Serial.println("turn: turn stepper at set step duration");
  Serial.println("pause: stop stepper without disabling driver");
  Serial.println("stop: disable stepper driver");
  Serial.println("help: print commands list");
}

void setup() {
  // put your setup code here, to run once:
  pinMode(dirpin, OUTPUT);
  pinMode(steppin, OUTPUT);
  pinMode(enablepin, OUTPUT);
  digitalWrite(enablepin, HIGH);

  // always make your code light up the LED to demonstrate that the teensy is working
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println("Teensy is powered and waiting for commands.");
  printCommands();
  stepTimer = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) { // if a message was received
    Serial.readBytesUntil(10, serialBuffer, 100); // grab the whole message until end of line
    char *restOfMessage = serialBuffer;
    char *msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // grab everything until a space

    if (String(msgElem) == "enable") {
      digitalWrite(enablepin, LOW); // enable it
      Serial.println("stepper power is enabled");
    }
    else if (String(msgElem) == "disable") {
      digitalWrite(enablepin, HIGH); // enable it
      Serial.println("stepper power is disabled");
    }
    else if (String(msgElem) == "up") {
      digitalWrite(dirpin, LOW); // enable it
      Serial.println("stepper going up");
    }
    else if (String(msgElem) == "down") {
      digitalWrite(dirpin, HIGH); // enable it
      Serial.println("stepper going down");
    }
    else if (String(msgElem) == "turn") {
      isTurning = true;
      Serial.print("stepper is stepping every ");
      Serial.print(stepInterval);
      Serial.println(" microseconds");
    }
    else if (String(msgElem) == "pause") {
      isTurning = false;
      Serial.println("stepper is paused but still powered");
    }
    else if (String(msgElem) == "stop") {
      digitalWrite(enablepin, HIGH);
      Serial.println("emergency stop");
    }

    else if (String(msgElem) == "duration") {
      char *msgElem = strtok_r(restOfMessage, " ", &restOfMessage); // grab everything until a space
      if (atoi(msgElem) == 0) {
        Serial.println("stepper duration not changing");
      }
      else {
        stepInterval = atoi(msgElem);
        Serial.print("new step duration is ");
        Serial.print(stepInterval);
        Serial.println(" microseconds");
      }
    }

    else if (String(msgElem) == "help") {
      printCommands();
    }
    else {
      Serial.println("unknown message, please try again");
    }
    
    memset(serialBuffer, 0, 100); // clear the buffer
  }

  if (isTurning) {
    if (stepTimer > stepInterval) {
      digitalWrite(steppin, HIGH);
      digitalWrite(steppin, LOW);
      stepTimer = 0;
    }
  }
}
