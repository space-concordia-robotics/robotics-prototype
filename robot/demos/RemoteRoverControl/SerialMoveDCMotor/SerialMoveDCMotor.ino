int DC_MOTOR_PIN = 11; // PWM 
int REST = 127;
int OFFSET = 30;
int CW = REST - OFFSET;
int CCW = REST + OFFSET;
int BUDGE_TIME = 100;
int BAUD_RATE = 9600;

void setup() 
{ 
    Serial.begin(BAUD_RATE);
    pinMode(DC_MOTOR_PIN, OUTPUT);
    analogWrite(DC_MOTOR_PIN, REST);
} 
 
 
void loop() 
{ 
  
    if (Serial.available()) {
        char cmd = Serial.read();
        Serial.println("cmd: ");
        Serial.println(cmd);

        if (cmd == 'w') {
            Serial.println("Moving Forward");
            // rotate clockwise
            analogWrite(DC_MOTOR_PIN, CCW);
            delay(BUDGE_TIME);
            // stop
            analogWrite(DC_MOTOR_PIN, REST);
            Serial.println("Stop");
        }
        else if (cmd == 's') {
            Serial.println("Moving Back");
            // rotate clockwise
            analogWrite(DC_MOTOR_PIN, CW);
            delay(BUDGE_TIME);
            // stop
            analogWrite(DC_MOTOR_PIN, REST);
            Serial.println("Stop");
        }
    }
}

