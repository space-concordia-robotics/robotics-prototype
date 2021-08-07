#include "DcMotor.h"

DcMotor::DcMotor(MotorNames name, uint8_t dirPin, uint8_t pwnPin, float gearRatio)
{
    this->name = name;
    this->dirPin = dirPin;
    this->pwmPin = pwnPin;
    // variables declared in RobotMotor require the this-> operator
    this -> gearRatio = gearRatio;
    this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time

}


void DcMotor::calcCurrentVelocity() {
//    Serial.print(dt);
//    Serial.print(" ");
//    Serial.print(encoderCount);
//    Serial.println(" ");
    uint32_t dt = InterruptHandler::getMotorDt(this->name);
    uint32_t encoderCount = InterruptHandler::getEncoderCount(this->name);

    if (dt <= 0 || encoderCount <= 0) {
        this->currentVelocity = 0;
    }
    else {
        this->currentVelocity = (float) (encoderCount * 60000000.0 * gearRatioReciprocal * encoderResolutionReciprocal / (float) (dt));
    }
    InterruptHandler::reset(this->name);
    encoderCount = 0;
    dt = 0;
    //  Serial.println(dt/1000);
}


void DcMotor::setVelocity() {

    this->calcCurrentVelocity();
    Serial.write((int)this->currentVelocity);
//    Serial.print(motorName);
    switch (this->desiredDirection) {
        case CW:
            digitalWrite(this->dirPin, HIGH);
            break;
        case CCW:
            digitalWrite(this->dirPin, LOW);
            break;
    }

    if (isOpenLoop) {

        int16_t output_pwm = desiredVelocity;
        Serial.write(output_pwm);
        analogWrite(this->pwmPin, output_pwm);
        Serial.write(1);

    }
    else if (!isOpenLoop) {
        // THIS LOOKS WRONG
        // makes sure the speed is within the limits set in the pid during setup
        if (desiredVelocity > 30) {
            this->desiredVelocity = 30;
        }
        else if (desiredVelocity < 0) {
            this->desiredVelocity = 0;
        }
        int16_t output_pwm = pidController->updatePID(this->currentVelocity, this->desiredVelocity);

//        Serial.print(output_pwm);
//        Serial.print(" ");

        analogWrite(this->pwmPin, fabs(output_pwm));

    }
    //this -> desiredVelocity = output_pwm;

    /* Acceleration limiter */
//    if (accLimit) {
//        final_output_pwm = output_pwm;
//        dt2 = micros() - prevTime2;
//
//        acc = ((float) output_pwm - (float) prev_output_pwm) / (float) dt2;
//
//        if (abs(acc) > 0.00051) {  // 0.00051 it the acceleration limit to go from 0 to full speed in 0.5 seconds. adjust this value for desired tuning
//            output_pwm = ((acc < 0) ? -1 : 1) * 0.000151 * dt2 + prev_output_pwm;
//        }
//        prevTime2 = micros();
//        prev_output_pwm = output_pwm;
//        analogWrite(pwmPin, output_pwm);
//    }
//    else analogWrite(pwmPin, output_pwm);

}

void DcMotor::setEnabled(bool isEnabled) {

}

void DcMotor::closeLoop() {

    this->isEncoderEnabled = true;
    maxOutputSignal = MAX_RPM_VALUE;
    minOutputSignal = MIN_RPM_VALUE;

    this->isOpenLoop = false;

}

void DcMotor::openLoop() {

    maxOutputSignal = MAX_PWM_VALUE;
    minOutputSignal = MIN_PWM_VALUE;

    this->isOpenLoop = true;
}

void DcMotor::attachEncoder(int encA, int encB, float encRes, void (*handler) (void)) {
    this->encoderPinA = encA;
    this->encoderPinB= encB;
    this->encoderResolution= encRes;
    this->encoderResolutionReciprocal = 1/encRes;


    pinMode(encB, INPUT_PULLUP);
    pinMode(encA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA), handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB), handler, CHANGE);
}

void DcMotor::initPidController(float kp, float ki, float kd) {
    this->pidController = new PidController(kp,ki,kd);
}


void DcMotor::updateDesiredVelocity(motor_direction desiredDirection, int16_t desiredVelocity) {
    this->desiredDirection = desiredDirection;
    this->desiredVelocity = desiredVelocity;
    Serial.write(desiredDirection);
    Serial.write(desiredVelocity);
    //setVelocity();
}

PidController *DcMotor::getPidController(void) const {
    return this->pidController;
}

motor_direction DcMotor::getDesiredDirection() const {
    return this->desiredDirection;
}

int16_t DcMotor::getDesiredVelocity() const {
    return this->desiredVelocity;
}

float DcMotor::getCurrentVelocity() const {
    return this->currentVelocity;
}

DcMotor::DcMotor() {

}
