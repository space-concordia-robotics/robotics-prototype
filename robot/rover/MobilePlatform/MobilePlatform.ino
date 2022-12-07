#include "Rover.h"
#include "commands/WheelsCommandCenter.h"

//#define DEBUG

#ifndef DEBUG
#define Serial Serial1
#endif

// Pins for Serial
const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t ENABLE_PIN = 15;
const uint8_t TRANSMIT_PIN = 14;

internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();

void attachMotors();
void attachServos();

void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);

}
void setup() {

    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(V_SENSE_PIN, INPUT);

    commandCenter->startSerial( RX_TEENSY_3_6_PIN,TX_TEENSY_3_6_PIN, ENABLE_PIN, TRANSMIT_PIN);

    attachMotors();
    attachServos();

    // Here different parameters of how the system should behave can be set
    Rover::systemStatus.is_throttle_timeout_enabled = true;
}

void loop() {
    if (Serial.available() > 0) {
        blink();
        commandCenter->readCommand();
    }
    // Sort of un-used at the moment, but this can periodically transmit messages to the OBC which can define the status
    // of the rover.

    // The rover will update its own current velocity according to a time interval which is measured from when it
    // last moved (in moveRover() )
    if ((millis() - Rover::systemStatus.last_velocity_adjustment) > ACCELERATION_RATE) {
        Rover::updateWheelVelocities();
    }
    // If the rover has not been commanded to move which a time interval, it must  be stopped (security risk).
    if (Rover::systemStatus.is_throttle_timeout_enabled &&
        ((millis() - Rover::systemStatus.last_move) > ROVER_MOVE_TIMEOUT)) {
        Rover::decelerateRover();
    }

    // In case there are any messages queued in the transmit buffer, they should be sent.
    commandCenter->sendMessage();
}

void attachServos(){
    Rover::attachServo(CENTER_BACK_1_SERVO,CB_1_SERVO);
    Rover::attachServo(CENTER_BACK_2_SERVO,CB_2_SERVO);
}

void attachMotors(){
    Motor::attachMotor(FRONT_RIGHT,M1_FR_DIR,M1_FR_PWM);
    Motor::attachMotor(MIDDLE_RIGHT,M2_MR_DIR,M2_MR_PWM);
    Motor::attachMotor(REAR_RIGHT,M3_RR_DIR,M3_RR_PWM);

    Motor::attachMotor(FRONT_LEFT,M4_FL_DIR,M4_FL_PWM);
    Motor::attachMotor(MIDDLE_LEFT,M5_ML_DIR,M5_ML_PWM);
    Motor::attachMotor(REAR_LEFT,M6_RL_DIR,M6_RL_PWM);
}
void WheelsCommandCenter::stopMotors() {
    Rover::stopMotors();
}

void WheelsCommandCenter::moveRover(const float & linear_y,const float & omega_z) {
    Rover::moveRover(linear_y,omega_z);
}

void WheelsCommandCenter::moveServo(const uint8_t & servoID, const uint8_t & angle) {
    Rover::moveServo((ServoNames)servoID,angle);
}
void WheelsCommandCenter::moveWheel(const uint8_t& wheelNumber,const uint8_t& direction,const uint8_t& speed) {
    Rover::moveWheel((MotorNames)wheelNumber,direction,speed);
}

void WheelsCommandCenter::pingWheels(void) {
    internal_comms::Message* message = commandCenter->createMessage(1, 0, nullptr);
    commandCenter->sendMessage(*message);
}

void WheelsCommandCenter::getBatteryVoltage() {

    //convert to 3.3V reference from analog values (3.3/1023=0.003225806)
    auto vsense = (float)analogRead(V_SENSE_PIN) * 0.003225806f;

    float vbatt = vsense * 6.0f;
    auto* buffer = (uint8_t *) malloc(4);

    float2bytes(buffer,vbatt);

    internal_comms::Message* message = commandCenter->createMessage(
            COMMAND_GET_BATTERY_VOLTAGE, sizeof(buffer), buffer);
    commandCenter->sendMessage(*message);
    free(buffer);
}
