int RF_PWM = 3; // 2
int RM_PWM = 5; // 3
int RB_PWM = 6; // 4
int LF_PWM = 9; // 5
int LM_PWM = 10; // 6
int LB_PWM = 11; // 7

int currentSpeed = 255;
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;
const int BUDGE_TIME = 100;

unsigned int prevRead = millis();

//@TODO: merge with actual motor drive code

void setup() {
    Serial.begin(9600);
    pinMode(RF_PWM, OUTPUT);
    pinMode(RM_PWM, OUTPUT);
    pinMode(RB_PWM, OUTPUT);
    pinMode(LF_PWM, OUTPUT);
    pinMode(LM_PWM, OUTPUT);
    pinMode(LB_PWM, OUTPUT);
}

void loop() {
    if (millis() - prevRead > BUDGE_TIME) {
        // stop turning
        analogWrite(RF_PWM, 0);
        analogWrite(RM_PWM, 0);
        analogWrite(RB_PWM, 0);
        analogWrite(LF_PWM, 0);
        analogWrite(LM_PWM, 0);
        analogWrite(LB_PWM, 0);
        
        if (Serial.available()) {
          char cmd = Serial.read();
        
          // for testing purposes
          // 255 will represent forward and 100 backward
          if (cmd == 'w') {
              Serial.println("Forward");
              analogWrite(RF_PWM, currentSpeed);
              analogWrite(RM_PWM, currentSpeed);
              analogWrite(RB_PWM, currentSpeed);
              analogWrite(LF_PWM, currentSpeed);
              analogWrite(LM_PWM, currentSpeed);
              analogWrite(LB_PWM, currentSpeed);
          } else if (cmd == 's') {
              analogWrite(RF_PWM, ceil(currentSpeed*0.5));
              analogWrite(RM_PWM, ceil(currentSpeed*0.5));
              analogWrite(RB_PWM, ceil(currentSpeed*0.5));
              analogWrite(LF_PWM, ceil(currentSpeed*0.5));
              analogWrite(LM_PWM, ceil(currentSpeed*0.5));
              analogWrite(LB_PWM, ceil(currentSpeed*0.5));
              Serial.println("Backward");
          } else if (cmd == 'a') {
              Serial.println("Left");
              analogWrite(RF_PWM, 0);
              analogWrite(RM_PWM, 0);
              analogWrite(RB_PWM, 0);
              analogWrite(LF_PWM, currentSpeed);
              analogWrite(LM_PWM, currentSpeed);
              analogWrite(LB_PWM, currentSpeed);
          } else if (cmd == 'd') {
              Serial.println("Right");
              analogWrite(RF_PWM, currentSpeed);
              analogWrite(RM_PWM, currentSpeed);
              analogWrite(RB_PWM, currentSpeed);
              analogWrite(LF_PWM, 0);
              analogWrite(LM_PWM, 0);
              analogWrite(LB_PWM, 0);
          }
          // decrease speed 
          else if (cmd == 'l') {
              if (currentSpeed > MIN_SPEED) {
                  Serial.print("Increasing speed to: ");
                  Serial.println(--currentSpeed);
              } else {
                  Serial.print("Already at maxspeed");
                  Serial.println(currentSpeed);
              }
          }
          // increase speed
          else if (cmd == 'm') {
              if (currentSpeed < MAX_SPEED) {
                  Serial.print("Decreasing speed to: ");
                  Serial.println(++currentSpeed);
              } else {
                  Serial.print("Already at minspeed: ");
                  Serial.println(currentSpeed);
              }
          }
          else if (cmd == 'p') {
              Serial.println("Pong");
          }
        }
        
        prevRead = millis();
    }
}
