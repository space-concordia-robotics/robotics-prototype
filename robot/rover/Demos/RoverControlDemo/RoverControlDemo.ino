// TODO: frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor
int motor = 11;
int REST = 127;
int OFFSET = 40;
int CW = REST + OFFSET;
int CCW = REST - OFFSET;
int BUDGE_TIME = 1000;
unsigned long startTime = 0;
unsigned long currentTime = 0;

void setup(){
    pinMode(motor, OUTPUT);
    analogWrite(motor, REST);
}
void loop(){
    
    // range: 0 to 255
    // 127 is resting state
    
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 'w') {
            currentTime = startTime = millis();
            analogWrite(motor, CW);
            
            while (currentTime - startTime < BUDGE_TIME) {
                currentTime = millis();
            }
            
        }
        else if (cmd == 's') {
            currentTime = startTime = millis();
            analogWrite(motor, CCW);
            
            while (currentTime - startTime < BUDGE_TIME) {
                currentTime = millis();
            }
        }
        
        analogWrite(motor, REST);
    }
}

