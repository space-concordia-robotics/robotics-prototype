// todo: finish everything, make sure to use the right pins for the encoders
class Motor(char motorType) {
	public:
		void update();
		
		void setMotorSpeed();
		void setDestinationAngle();
		float getCurrentAngle();
	private:
		char motorType;
		float motorAngle;
		volatile int encoderCount;
		
		void changeMotorAngle(char direction, char speed);
		void Motor(char motorType);
};

Motor::Motor(char motorCode){
	this.motorCode = motorCode;
	switch(motorCode){
		case MOTOR1:
			this.motorType=STEPPER_MOTOR;
			attachInterrupt(M1_ENCODER_A, m1_encoder_interrupt, CHANGE);
			attachInterrupt(M1_ENCODER_B, m1_encoder_interrupt, CHANGE);
			break;
		case MOTOR2:
			attachInterrupt(M2_ENCODER_A, m2_encoder_interrupt, CHANGE);
			attachInterrupt(M2_ENCODER_B, m2_encoder_interrupt, CHANGE);
			break;
		case MOTOR3:
			this.motorType=STEPPER_MOTOR;
			attachInterrupt(M3_ENCODER_A, m3_encoder_interrupt, CHANGE);
			attachInterrupt(M3_ENCODER_B, m3_encoder_interrupt, CHANGE);
			break;
		case MOTOR4:
			this.motorType=STEPPER_MOTOR;
			attachInterrupt(M4_ENCODER_A, m4_encoder_interrupt, CHANGE);
			attachInterrupt(M4_ENCODER_B, m4_encoder_interrupt, CHANGE);
			break;
		case MOTOR5:
			this.motorType=SERVO_MOTOR;
			break;
		case MOTOR6:
			this.motorType=SERVO_MOTOR;
			break;
	}
}

Motor::changeMotorAngle(char direction, char speed){
	switch(this.motorType){
		case DC_MOTOR:
			break;
		case STEPPER_MOTOR:
			break;
		case SERVO_MOTOR:
			if(direction==CLOCKWISE){
				//pwm signal one way
			}
			if(direction==COUNTER_CLOCKWISE){
				//pwm signal other way
			}
			break;
	}
}

void m1_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  //read all bits on D register. shift ro right
  //so pin 2 and 3 are now the lowest bits
  //then AND this with 0X03 (0000 0011) to zero everything else
  //then OR this with the last encoder state to get a 4 byte number
  oldEncoderState |= ((PIND >> 2) & 0x03);
  //AND this number with 0X0F to make sure its a
  //4 bit unsigned number (0 to 15 decimal)
  //then use that number as an index from the array to add or deduct a 1 from the
  //count
  motor1.encoderCount += dir[(oldEncoderState & 0x0F)];
}

void m2_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  oldEncoderState |= ((PIND >> 2) & 0x03);
  motor2.encoderCount += dir[(oldEncoderState & 0x0F)];
}

void m3_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  oldEncoderState |= ((PIND >> 2) & 0x03);
  motor3.encoderCount += dir[(oldEncoderState & 0x0F)];
}

void m4_encoder_interrupt(){
  static unsigned int oldEncoderState = 0;
  oldEncoderState <<= 2;  //move by two bits (multiply by 4);
  oldEncoderState |= ((PIND >> 2) & 0x03);
  motor4.encoderCount += dir[(oldEncoderState & 0x0F)];
}