int motor = 11;


void setup(){
	pinMode(motor, OUTPUT);
}
void loop(){
	// range: 0 to 255
	// 127 is resting state
	analogWrite(motor,110);
}
