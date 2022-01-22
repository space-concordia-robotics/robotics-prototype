#include "Arduino.h"

#define NUM_MOTORS 6

#define M6_RL_PWM 2
#define M5_ML_PWM 3
#define M4_FL_PWM 4

#define M3_RR_PWM 5
#define M2_MR_PWM 6
#define M1_FR_PWM 7

#define M6_RL_DIR 26
#define M5_ML_DIR 25
#define M4_FL_DIR 24

#define M3_RR_DIR 12
#define M2_MR_DIR 11
#define M1_FR_DIR 8

#define M6_RL_A 27
#define M6_RL_B 28

#define M5_ML_A 33
#define M5_ML_B 34

#define M4_FL_A 31
#define M4_FL_B 32

#define M3_RR_A 29
#define M3_RR_B 30

#define M2_MR_A 37
#define M2_MR_B 38

#define M1_FR_A 35
#define M1_FR_B 36

enum MotorNames{
    FRONT_RIGHT=0,
    MIDDLE_RIGHT=1,
    REAR_RIGHT=2,
    FRONT_LEFT=3,
    MIDDLE_LEFT=4,
    REAR_LEFT=5
};
typedef struct DcMotorState{
    MotorNames id;

    uint8_t desired_velocity;
    uint8_t current_velocity;

    float actual_velocity;
    uint8_t max_pwm_value;

    uint8_t desired_direction;

    uint8_t dir_pin;
    uint8_t pwm_pin;

} DcMotorState;

DcMotorState motorList[6];

void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);
}

void attachMotor(const MotorNames &motorID, const uint8_t& dirPin,const uint8_t& pwmPin) {
    motorList[motorID] = {};
    motorList[motorID].dir_pin = dirPin;
    motorList[motorID].pwm_pin = pwmPin;
    motorList[motorID].id = motorID;

    motorList[motorID].desired_velocity = 0;
    motorList[motorID].current_velocity = 0;

    motorList[motorID].max_pwm_value = 255;

    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
}
void attachMotors(){

    attachMotor(FRONT_RIGHT,M1_FR_DIR,M1_FR_PWM);
    attachMotor(MIDDLE_RIGHT,M2_MR_DIR,M2_MR_PWM);
    attachMotor(REAR_RIGHT,M3_RR_DIR,M3_RR_PWM);

    attachMotor(FRONT_LEFT,M4_FL_DIR,M4_FL_PWM);
    attachMotor(MIDDLE_LEFT,M5_ML_DIR,M5_ML_PWM);
    attachMotor(REAR_LEFT,M6_RL_DIR,M6_RL_PWM);
}

void accelerateWheel(uint8_t id,uint32_t rate, uint8_t direction){
    int currentPWM = 60;
    uint32_t startTime = millis();
    auto& motor = motorList[id];
    analogWrite(motor.dir_pin,direction);

    if( (millis() - startTime) > rate){
        currentPWM++;
        Serial.print(currentPWM);
        analogWrite(motor.pwm_pin,currentPWM);
        if(currentPWM >= 255){
            delay(2000);
            blink();
            return;
        }
    }
}
void setup(){
    Serial.begin(9800);

    blink();

    attachMotors();

    accelerateWheel(FRONT_RIGHT,20,0);
    accelerateWheel(FRONT_RIGHT,20,1);

    accelerateWheel(MIDDLE_RIGHT,20,0);
    accelerateWheel(MIDDLE_RIGHT,20,1);

    accelerateWheel(REAR_RIGHT,20,0);
    accelerateWheel(REAR_RIGHT,20,1);

    accelerateWheel(FRONT_LEFT,20,0);
    accelerateWheel(FRONT_LEFT,20,1);

    accelerateWheel(MIDDLE_LEFT,20,0);
    accelerateWheel(MIDDLE_LEFT,20,1);

    accelerateWheel(REAR_LEFT,20,0);
    accelerateWheel(REAR_LEFT,20,1);


}

void loop(){
    while(true){
        blink();
    }
}