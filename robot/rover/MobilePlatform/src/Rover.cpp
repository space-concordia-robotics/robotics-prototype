#include "Rover.h"

void Rover::roverVelocityCalculator(void) {

    this->rightLinearVelocity = (float)(motorList[RIGHT_BACK].getDesiredDirection() * motorList[RIGHT_FRONT].getCurrentVelocity() +
            motorList[RIGHT_MIDDLE].getDesiredDirection() * motorList[RIGHT_MIDDLE].getCurrentVelocity() +
            motorList[RIGHT_BACK].getDesiredDirection() * motorList[RIGHT_BACK].getCurrentVelocity()) * radius * piRad;

    this->leftLinearVelocity = (float)(motorList[LEFT_FRONT].getDesiredDirection() * motorList[LEFT_FRONT].getCurrentVelocity() +
            motorList[LEFT_MIDDLE].getDesiredDirection() * motorList[LEFT_MIDDLE].getCurrentVelocity() +
            motorList[LEFT_BACK].getDesiredDirection() * motorList[LEFT_BACK].getCurrentVelocity()) * radius * piRad;

    this->linearVelocity = (rightLinearVelocity - leftLinearVelocity)  / 6;
    this->rotationalVelocity = (leftLinearVelocity + rightLinearVelocity) / wheelBase;
}


// Emergency stop all motors
void Rover::stopMotors(void) {
    this->steerRover(0, 0); // Set all motors throttle and steering to 0
}

void Rover::enableAllMotors(uint8_t state) {
    for (auto& motor : motorList) {
        motor.setEnabled(state);
    }
}
void Rover::attachMotor(MotorNames name,uint8_t dir, uint8_t pwn, float gr) {
    motorList[motorCount] = DcMotor(name,dir,pwn,gr);
}

void Rover::closeAllMotorLoop() {

    this->maxOutputSignal = MAX_RPM_VALUE;
    this->minOutputSignal = MIN_RPM_VALUE;

    if(this->areMotorsEnabled){
        this->stopMotors();
    }
    for (auto& motor : motorList) {
        motor.closeLoop();
    }
}
void Rover::openAllMotorLoop() {

    maxOutputSignal = MAX_PWM_VALUE;
    minOutputSignal = MIN_PWM_VALUE;

    if(this->areMotorsEnabled){
        this->stopMotors();
    }
    for (auto& motor : motorList) {
        motor.openLoop();
    }
}


void Rover::moveWheel(uint8_t wheelName, const int16_t wheelPWM) {
        const auto& direction= (wheelPWM < 0) ? CW : CCW;

        this->isSteering = 0; // From Globals.h

        motorList[wheelName].updateDesiredVelocity(direction , abs(wheelPWM));

}

void Rover::updateWheelsVelocity(){
    for(auto& motor : motorList){
        motor.updateDesiredVelocity(motor.getDesiredDirection(), motor.getDesiredVelocity());
    }
}

void Rover::steerRover(int8_t throttle,int8_t steering){
    float multiplier = mapFloat(fabs(steering), 0, MAX_INPUT_VALUE, 1, -1);
    float leadingSideAbs = mapFloat(fabs(throttle), 0, MAX_INPUT_VALUE, 0, maxOutputSignal);
    float trailingSideAbs = leadingSideAbs * multiplier;

    int dir = 1;
    if (throttle >= 0) dir = 1;
    else if (throttle < 0) dir = -1;

    if (steering < 0) { // turning left
        this->desiredVelocityRight = leadingSideAbs * dir;
        this->desiredVelocityLeft = trailingSideAbs * dir;
    }
    else { // turning right
        this->desiredVelocityRight = trailingSideAbs * dir;
        this->desiredVelocityLeft = leadingSideAbs * dir;
    }

    if (desiredVelocityLeft > 0)
        this->leftMotorDirection = CCW;
    else
        this->leftMotorDirection = CW;

    if (desiredVelocityRight < 0)
        this->rightMotorDirection = CCW;
    else
        this->rightMotorDirection = CW;

    motorList[RIGHT_FRONT].updateDesiredVelocity(rightMotorDirection, fabs(desiredVelocityRight));
    motorList[RIGHT_MIDDLE].updateDesiredVelocity(rightMotorDirection, fabs(desiredVelocityRight));
    motorList[RIGHT_BACK].updateDesiredVelocity(rightMotorDirection, fabs(desiredVelocityRight));
    motorList[LEFT_FRONT].updateDesiredVelocity(leftMotorDirection, fabs(desiredVelocityRight));
    motorList[LEFT_MIDDLE].updateDesiredVelocity(leftMotorDirection, fabs(desiredVelocityLeft));
    motorList[LEFT_BACK].updateDesiredVelocity(leftMotorDirection, fabs(desiredVelocityLeft));

    //this->sinceThrottle = 0;

}



void Rover::setMotorPidGains(MotorNames motor, float Kp, float Ki, float Kd) {
    motorList[motor].getPidController()->setGainConstants(Kp,Ki,Kd);
}

//! !FB, @FS, #RB, $RS
void Rover::controlCameraMotors(ServoNames servoID, uint16_t angle) {
    servoList[servoID].write(angle);
}

void Rover::setMotorsEnabled(uint8_t state) {
    this->areMotorsEnabled = state;
}

uint8_t Rover::getMotorsEnabled() const {
    return areMotorsEnabled;
}

float Rover::getLinearVelocity() const {
    return this->linearVelocity;
}

float Rover::getRotationalVelocity() const {
    return this->linearVelocity;
}

void Rover::setThrottleTimeout(const uint8_t & state) {
    this->throttleTimeout= state;
}

uint8_t Rover::getSteeringEnabled() const {
    return this->isSteering;
}

void Rover::writeServo(ServoNames name, int16_t value) {
    //servoList.at(name).write(value);
}

void Rover::setSteeringEnabled(const uint8_t & state) {
    this->isSteering = state;
}

void Rover::attachServo(ServoNames name, uint8_t pin) {

    servoList[servoCount++] = Servo();
    servoList[name].attach(pin);
}

void Rover::setAccelerationLimiter(uint8_t state) {
    this->accelerationLimiterEnabled = state;
}

uint8_t Rover::isThrottleTimeoutEnabled() const {
    return this->throttleTimeout;
}

Rover::Rover() {

}



