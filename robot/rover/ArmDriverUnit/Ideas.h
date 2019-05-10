/* ROS stuff for beginning of the sketch
// develmode1 actually isn't for ros... i will have to change things if i want ros over usb
#ifdef DEVEL_MODE_1 // using the USB port
//ros::NodeHandle nh;
#elif defined(DEBUG_MODE) || defined(USER_MODE) // using hardware serial (Serial1 in this case)
// the following code allows you to choose the hardware serial port
  class NewHardware : public ArduinoHardware {
  public:
  long baud = 57600;
  NewHardware():ArduinoHardware(&Serial1, baud){}; // place the serial port of your choosing (1 to 6)
  };
  ros::NodeHandle_<NewHardware> nh;
// otherwise just use this
ros::NodeHandle nh;
#endif
#ifdef ENABLE_ROS
void messageCallback(const std_msgs::String& cmd_message) {
  msgReceived = true;
  int i = 0;
  while (cmd_message.data[i] != '\0') {
    serialBuffer[i] = cmd_message.data[i];
  }
  Parser.parseCommand(motorCommand, serialBuffer);
  if (Parser.verifCommand(motorCommand)) {
    msgIsValid = true;
  }
  memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
}
ros::Subscriber<std_msgs::String> cmdSubscriber("arm_command", &messageCallback);

// these hold information that is sent from the teensy to ros
char m1FrameId[] = "/m1_angle";
char m2FrameId[] = "/m2_angle";
char m3FrameId[] = "/m3_angle";
char m4FrameId[] = "/m4_angle";
char m5FrameId[] = "/m5_angle";
char m6FrameId[] = "/m6_angle";
sensor_msgs::JointState m1_angle_msg;
sensor_msgs::JointState m2_angle_msg;
sensor_msgs::JointState m3_angle_msg;
sensor_msgs::JointState m4_angle_msg;
sensor_msgs::JointState m5_angle_msg;
sensor_msgs::JointState m6_angle_msg;
sensor_msgs::JointState angleMessages[NUM_MOTORS] = {m1_angle_msg, m2_angle_msg, m3_angle_msg, m4_angle_msg, m5_angle_msg, m6_angle_msg};
ros::Publisher pub_m1("m1_joint_state", &m1_angle_msg);
ros::Publisher pub_m2("m2_joint_state", &m2_angle_msg);
ros::Publisher pub_m3("m3_joint_state", &m3_angle_msg);
ros::Publisher pub_m4("m4_joint_state", &m4_angle_msg);
ros::Publisher pub_m5("m5_joint_state", &m5_angle_msg);
ros::Publisher pub_m6("m6_joint_state", &m6_angle_msg);
#endif
 */
/* // idea for stepper checks in open loop
  if (sinceStepperCheck >= STEPPER_CHECK_INTERVAL) { // for open loop quasi closed loop control
    // this code could (should?) also disable power
  // if (motor1.movementDone) motor1.disablePower();
  // if (motor3.movementDone) motor3.disablePower();
  // if (motor4.movementDone) motor4.disablePower();
   this code could (should?) determine when servo/dc motor movement is done

  if ( fabs(motor2.desiredAngle - motor2.calcCurrentAngle() ) < motor2.pidController.jointAngleTolerance) {
  motor2.movementDone = true;
  }


  if ( fabs(motor5.desiredAngle - motor5.calcCurrentAngle() ) < motor5.pidController.jointAngleTolerance) {
  motor5.movementDone = true;
  }
  if ( fabs(motor6.desiredAngle - motor6.calcCurrentAngle() ) < motor6.pidController.jointAngleTolerance) {
  motor6.movementDone = true;
  }

  // all of this code should probably go into a function called "calculate motor steps" or something...
  // this code is very similar to what happens above when it decides which motor should turn after receiving a command
  // this code also assumes that the correct amount of steps will take it to the right spot
  // this means that it doesn't account for faulty angle calculations from the encoder or the motor resolution...

  //motor4
  int m4remainingSteps = motor4.numSteps - motor4.stepCount;
  //float m4imaginedAngle = motor4.stepCount * motor4.stepResolution * motor4.gearRatioReciprocal;
  //float m4actualAngle = motor4.calcCurrentAngle();
  float m4imaginedRemainingAngle = m4remainingSteps * motor4.stepResolution * motor4.gearRatioReciprocal; // how far does the motor think it needs to go
  float m4actualRemainingAngle = motor4.desiredAngle - motor4.calcCurrentAngle(); // how far does it actually need to go
  float m4discrepancy = m4actualRemainingAngle - m4imaginedRemainingAngle ;
  //motor3
  int remainingSteps = motor3.numSteps - motor3.stepCount;
  //float imaginedAngle = motor3.stepCount * motor3.stepResolution * motor3.gearRatioReciprocal;
  //float actualAngle = motor3.calcCurrentAngle();
  float imaginedRemainingAngle = remainingSteps * motor3.stepResolution * motor3.gearRatioReciprocal; // how far does the motor think it needs to go
  float actualRemainingAngle = motor3.desiredAngle - motor3.calcCurrentAngle(); // how far does it actually need to go
  float discrepancy = actualRemainingAngle - imaginedRemainingAngle ;

  // UART_PORT.print(imaginedAngle); UART_PORT.println(" imagined angle");
  // UART_PORT.print(actualAngle); UART_PORT.println(" actual angle");
  // UART_PORT.print(imaginedRemainingAngle); UART_PORT.println(" imagined remaining angle");
  // UART_PORT.print(actualRemainingAngle); UART_PORT.println(" actual remaining angle");
  // UART_PORT.print(discrepancy); UART_PORT.println(" degrees behind expected position");

  // the stepper interrupt could occur during this calculation, so maybe there should be a different angle tolerance here
  // that said at the moment it's 2 degrees which is bigger than the max step angle of the motor
  // keep in mind that 2 degrees for the joint is different from 2 degrees for the motor shaft
  if (fabs(discrepancy) > motor3.pidController.jointAngleTolerance) {
  UART_PORT.println("discrepancy is too high and motor is moving, adjusting step number");
  // it's possible the check happens during movement, but there needs to be ample distance to move
  if (!motor3.movementDone) {
    // if actualRemainingAngle is negative it means the arm moved way further than it should have
    if (actualRemainingAngle < 0) motor3.movementDone = true; // abort
    else if (actualRemainingAngle > motor3.pidController.jointAngleTolerance) {
      UART_PORT.println("enough angle between current position and desired position to adjust during movement");
      // the adjustment is simple if the motor is already moving in the right direction but what happens when a direction change needs to occur?
      // the motor interrupt assumes the step count and direction are unchanged!!!!
      motor3.numSteps += discrepancy * motor3.gearRatio / motor3.stepResolution; // add the number of steps required to catch up or skip
      //numsteps gets updated but imagined angle doesnt...?
    }
    else UART_PORT.println("not enough angle between current position and desired position to adjust during movement, waiting for movement to end");
  }
  else { // it's possible the check happens when the motor is at rest
    UART_PORT.println("discrepancy is too high and motor is done moving, adjusting step number");
    // it's possible the angle is too far in either direction, so this makes sure that it goes to the right spot
    if (discrepancy >= 0) motor3.rotationDirection = 1;
    else discrepancy = -1;
    motor3.enablePower();
    motor3.numSteps = fabs(discrepancy) * motor3.gearRatio / motor3.stepResolution; // calculate the number of steps to take
    motor3.movementDone = false;
  }
  }
  sinceStepperCheck = 0;
  }
*/

/* create wrappers to circumvent C++ refusing to attach instance-dependent interrupts inside a class */
/*
  void m1WrapperISR(void) {
  motor1.encoder_interrupt();
  }
  void m2WrapperISR(void) {
  motor2.encoder_interrupt();
  }
  void m3WrapperISR(void) {
  motor3.encoder_interrupt();
  }
  void m4WrapperISR(void) {
  motor4.encoder_interrupt();
  }
  //void m5WrapperISR(void){ motor5.encoder_interrupt(); }
  //void m6WrapperISR(void){ motor6.encoder_interrupt(); }
*/
/*
       attachInterrupt(motor1.encoderPinA, m1WrapperISR, CHANGE);
      attachInterrupt(motor1.encoderPinB, m1WrapperISR, CHANGE);

      attachInterrupt(motor2.encoderPinA, m2WrapperISR, CHANGE);
      attachInterrupt(motor2.encoderPinB, m2WrapperISR, CHANGE);

      attachInterrupt(motor3.encoderPinA, m3WrapperISR, CHANGE);
      attachInterrupt(motor3.encoderPinB, m3WrapperISR, CHANGE);

      attachInterrupt(motor4.encoderPinA, m4WrapperISR, CHANGE);
      attachInterrupt(motor4.encoderPinB, m4WrapperISR, CHANGE);
*/
/*
       attachInterrupt(motor5.encoderPinA, m5WrapperISR, CHANGE);
  attachInterrupt(motor5.encoderPinB, m5WrapperISR, CHANGE);

  attachInterrupt(motor6.encoderPinA, m6WrapperISR, CHANGE);
  attachInterrupt(motor6.encoderPinB, m6WrapperISR, CHANGE);
*/

/* // for multiple steppers in same timer interrupt (incomplete, complicated to implement)
  void stepperInterrupt(void) {
  static int nextInterval = 0;
  int i = 3;
  // code to decide which motor to turn goes here, or code just turns all motors
  if (!stepperArray[i - 1].movementDone) {
    // code to decide how fast the motor will turn and in which direction
    int dir = CLOCKWISE;
    nextInterval = 25000;
    //set a minimum step period somewhere
    //nextInterval = stepperArray[i-1].pidController.updatePID(stepperArray[i-1].currentAngle, stepperArray[i-1].desiredAngle);
    stepperArray[i - 1].singleStep(dir);
    stepperTimer.update(nextInterval);
  }
  }
*/

/* // takes the key (speed, time, etc) and updates?outputs? the next value
  // this one is for unsigned ints, would need to define separate ones for other data types?
  parseKey(String key, unsigned int valueOut, String delimeter, unsigned int minVal, unsigned int maxVal) {
  char *msgElem = strtok_r(NULL, delimeter, &restOfMessage); // find the next message element
  if (String(msgElem) == key) { // msgElem is a char array so it's safer to convert to string first
    msgElem = strtok_r(NULL, delimeter, &restOfMessage); // find the next message element (time in seconds)
    unsigned int tempVar = atoi(msgElem); // converts to int
    //if (tempTimeVar <= MAX_BUDGE_TIME && tempTimeVar >= MIN_BUDGE_TIME) { // don't allow budge movements to last a long time
    if (tempVar <= maxVal && tempVar >= minVal) {
      //motorCommand.whichTime = tempTimeVar;
      valueOut = tempVar;
      Serial.print("parsed time interval "); Serial.print(motorCommand.whichTime); Serial.println("ms");
    }
  }
  }
*/

// NICK STUFF
/* variable definitions
  bool msgCheck = false;
  bool msgState = false;

  const int ledPin = 13; // note to nick: LED_BUILTIN is predefined in arduino C++ so just use that instead
  unsigned long int previousMillis = 0; //stores previous time (in millis) LED was updated // note to nick: this variable name is too generic given how many other variables are used. Use more specific variable names, and also I recommend using elapsedMillis objects like I do to keep my code consistent
  int ledState = LOW;

  const int goodBlinkCounter = 4;
  const int badBlinkCounter = 12;
  const int goodBlinkInterval = 250;
  const int badBlinkInterval = 100;
  bool complete = false; // note to nick: this variable name is way too generic to just be lying around in the middle of all my code. please use something more specific
*/
/* if msgReceived
      msgCheck = true; //Setting message check value to TRUE as a message is received
      if (msgIsValid == true){
      msgState = true;
      }
      else{
      msgState = false;
      }
*/
/* in main loop outside of msgReceived
  if(msgCheck == true){
  if(msgState == true){
  msgCheck = Blink(goodBlinkInterval, goodBlinkCounter);
  }
  else {
  msgCheck = Blink(badBlinkInterval, badBlinkCounter);
  }
  }
  else {
  heartbeat();
  }
*/
/* function definitions
  bool Blink(const int ledInterval, int maxBlinks){
  static bool complete = false;
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= ledInterval){
    previousMillis = currentMillis;
    if(ledState == LOW){
      ledState = HIGH;
    } else{
      ledState = LOW;
    }
    digitalWrite(led, ledState);
    Serial.println(ledState);
    blinkCounter++;
  }
  if(blinkCounter == maxBlinks){
    complete = true;
    blinkCounter = 0;
  }
  return complete;
  }

  void heartbeat(){

  }
*/
// ABTIN STUFF
//setup(){
/*
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 1000;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= ((1 << CS11) | (1 << CS10));
  interrupts();

  //start of the heartbeat timer
  startTime = millis();

  }
*/
/* void ISR(TIMER1_COMPA_vect)
  {
  for (int i = 0; i < NUM_STEPPERS; i++) {
   volatile stepperInfo& s = steppers[i];

    if (s.currentAngle >= 360) {
     s.currentAngle = 0;
   }
   if (s.currentAngle < 0) {
     s.currentAngle = 360;
   }

    if ( abs(s.currentAngle - s.desiredAngle) > 1.7 && !s.movementDone  ) {
     s.stepFunc();
     s.currentAngle += s.dir;

    } else {
     s.movementDone = true;
   }

    if (steppers[0].movementDone && steppers[1].movementDone) {  //very dirty. we dont want to check manually.
     TIMER1_INTERRUPTS_OFF
   }
  }
  }
*/
/* void startMovement() {
  OCR1A = c0;
  TIMER1_INTERRUPTS_ON
  }
*/
