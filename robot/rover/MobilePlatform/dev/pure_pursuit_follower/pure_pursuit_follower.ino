/*
Pure pursuit path follower.
I think this is the latest code, but I'm not sure. It's been a while ago.
*/
#include <PID_v1.h>
#include <DeadReckoner.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>

const unsigned long BAUD_RATE = 115200;

// PINS
// See table on page below to find out wich pins are compatible with interrupts.
// For the Uno D2 and D3 are the only ones compatible.a
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;
const int BLUETOOTH_TX = 8;
const int BLUETOOTH_RX = 7;
// motor one
const int ENA = 5;
const int IN1 = 4;
const int IN2 = 6;
// motor two
const int ENB = 11;
const int IN3 = 10;
const int IN4 = 9;

// DEAD RECKONING CONSTANTS _____TODO: CHANGE TO LONG IN DEAD RECKONING LIBRARY____
volatile unsigned long leftTicks = 0, rightTicks = 0;
const int TICKS_PER_REV = 144; // number of ticks on the encoder per revolution
const double RADIUS = 29; // wheel radius in mm
const double LENGTH = 102; // wheel base length in mm

               // PURE PURSUIT CONSTANTS
               // TODO: LOOK_AHEAD = 50
const double LOOK_AHEAD = 250; // look ahead parameter l in mm

const unsigned long POSITION_COMPUTE_INTERVAL = 25; // millisecond
const unsigned long PURE_PURSUIT_COMPUTE_INTERVAL = 50; // millisecond
const unsigned long LOCATION_SEND_TIME_INTERVAL = 300; // millisecond

                             // Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevPurePursuitComputeTime = 0, prevLocationSendTime = 0;

void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

// PID PARAMETERS
// This is optional, but using PID will help the robot go straighter.
// Tip for tuning PID: https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
const double KP = 100;
const double KI = 0;
const double KD = 0;
const int PID_RANGE = 100;
const int PID_SAMPLE_TIME = 15; // milliseconds
double input = 0, output = 0, setpoint = 0;

DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);
PID pid(&input, &output, &setpoint, KP, KI, KD, DIRECT);
SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue bluetoothController(bluetoothSerial);

bool isTraversingPath = false;
int goalIndex;
int prevClosestPathIndex = 1;
int motorSpeed = 0;
int pathLength;

// Attaches interrupts
void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

// Weird bug with abs() returning negative so use this instead.
double absoluteVal(double num) {
  if (num < 0) return -num;
  return num;
}

double getDistance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Not the actual distance but faster performance. Sufficient to find the min or max.
double getDVal(double x1, double y1, double x2, double y2) {
  return absoluteVal(x2 - x1) + absoluteVal(y2 - y1);
}

void motorSetForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Returns the next closest path index.
int getClosestPathIndex(double xPos, double yPos, int fromIndex) {
  double closestPathDistance = 99999999;
  double xPath, yPath, dist;
  int closestPathIndex = NAN;

  // FIND THE NEXT CLOSEST PATH INDEX
  // !!! PUT BACK TO NORMAL 
  for (int i = 0; i < pathLength; i++) {
    xPath = bluetoothController.getPathX(i);
    yPath = bluetoothController.getPathY(i);
    dist = getDistance(xPath, yPath, xPos, yPos);
    if (dist < closestPathDistance) {
      closestPathDistance = dist;
      closestPathIndex = i;
    }
  }
  return closestPathIndex;
}

int getGoalPointIndex(double xPos, double yPos, int fromIndex) {
  double minDelta = 99999999;
  double distance, xPath, yPath;
  double d1, d2;
  int goalIndex = NAN;

  // Find the goal point
  for (int i = fromIndex; i < pathLength; i++) {
    xPath = bluetoothController.getPathX(i);
    yPath = bluetoothController.getPathY(i);

    distance = getDistance(xPos, yPos, xPath, yPath);
    double delta = abs(distance - LOOK_AHEAD);

    if (delta < minDelta) {
      minDelta = delta;
      goalIndex = i;
    }
    /*Serial.print("131\tdist: "); Serial.print(dist);
    Serial.print("\td1: "); Serial.print(d1);
    Serial.print("\tclosest: "); Serial.print(closestGoalDistance);
    Serial.print("\td2: "); Serial.println(d2);*/
  }
  return goalIndex;
}

void moveWheels(int motorSpeed, double diff) {
  int speedLeft, speedRight;

  // Set speed negation
  // pos diff -> turn left
  // neg diff -> turn right
  if (diff > 0) {
    speedLeft = map(diff, PID_RANGE, 0, 0, motorSpeed);
    speedRight = motorSpeed;
  }
  else {
    diff = -diff;
    speedLeft = motorSpeed;
    speedRight = map(diff, PID_RANGE, 0, 0, motorSpeed);
  }
  analogWrite(ENA, speedLeft);
  analogWrite(ENB, speedRight);
}

void doDeadReckoning() {
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
    // Computes the new angular velocities and uses that to compute the new position.
    // The accuracy of the position estimate will increase with smaller time interval until a certain point.
    deadReckoner.computePosition();
    prevPositionComputeTime = millis();
  }
}

bool doPurePursuit() {
  if (millis() - prevPurePursuitComputeTime > PURE_PURSUIT_COMPUTE_INTERVAL) {
    // 1. Find the current location of the vehicle.
    double xPos = deadReckoner.getX();
    double yPos = deadReckoner.getY();
    double phi = deadReckoner.getTheta() - PI / 2;

    // 2. Find the path point closest to the vehicle.
    int closestIndex = getClosestPathIndex(xPos, yPos, 0);

    // 3. Find the goal point.
    goalIndex = getGoalPointIndex(xPos, yPos, closestIndex);
    // Todo fix this
    double xGoal = bluetoothController.getPathX(goalIndex);
    double yGoal = bluetoothController.getPathY(goalIndex);

    // 4. Transform the goal point to vehicle coordinates.
    // First translate it such that the origin is the robot.
    xGoal -= xPos;
    yGoal -= yPos;
    // Second rotate the coordinate system such that the wheel axle points to the x-axis.
    double xGoalRelRobot = xGoal * cos(phi) + yGoal * sin(phi);

    // 5. Calculate the curvature.
    double curvature = 2 * xGoalRelRobot / LOOK_AHEAD;
    // double radius = 1 / curvature;

    // 6. Update the vehicle's position.
    // pos x in vehicle coordinates -> pos curvature -> turn right
    // neg x in vehicle coordinates -> neg curvature -> turn left
    // Ratio between left and right angular velocity of wheel to achieve desired curvature.
    // double ratio = (radius + LENGTH / 2) / (radius - LENGTH / 2);

    input = curvature;

    prevPurePursuitComputeTime = millis();

    // Debug
    //Serial.print("207\tci: "); Serial.print(closestIndex);
    //Serial.print("\tgi: "); Serial.print(goalIndex);
    //Serial.print("\txg: "); Serial.print(bluetoothController.getPathX(goalIndex));
    //Serial.print("\tyg: "); Serial.print(bluetoothController.getPathY(goalIndex));
    //Serial.print("\txp: "); Serial.print(xPos);
    //Serial.print("\typ: "); Serial.print(yPos);
    //Serial.print("\ttheta: "); Serial.print((phi + PI / 2)*180/PI);
    //Serial.print("\txGoalRelRobot: "); Serial.print(xGoalRelRobot);
    //Serial.print("\to: "); Serial.print(output);
    //Serial.print("\tc: "); Serial.println(curvature);

    // TODO: 7. Check if path has been traversed
    /* if (closestIndex == pathLength - 1 || goalIndex == pathLength - 1) {
    // !!! - put this back to true.
    return false;
    } */

    return true;
  }
  return false;
}

void sendLocationData() {
  if (millis() - prevLocationSendTime > LOCATION_SEND_TIME_INTERVAL) {
    double xPos = deadReckoner.getX();
    double yPos = deadReckoner.getY();
    double heading = deadReckoner.getTheta();
    double xGoal = bluetoothController.getPathX(goalIndex);
    double yGoal = bluetoothController.getPathY(goalIndex);
    bluetoothController.sendLocation(xPos, yPos, heading, xGoal, yGoal);

    // Debug
    //Serial.print("233\txSend: "); Serial.print(xPos);
    //Serial.print("233\tySend: "); Serial.print(yPos);
    //Serial.print("233\theading: "); Serial.print(heading);
    //Serial.print("233\tgoalIndex: "); Serial.println(goalIndex);
    prevLocationSendTime = millis();
  }
}

void printPath() {
  double x, y;

  // Debug
  Serial.println("PRINTING PATH");
  for (int i = 0; i < pathLength; i++) {
    x = bluetoothController.getPathX(i);
    y = bluetoothController.getPathY(i);
    // Debug
    Serial.print("244\ti: "); Serial.print(i);
    Serial.print("\tx: "); Serial.print(x);
    Serial.print("\ty: "); Serial.println(y);
  }
}

void doDebug() {
  double xPos = 11;
  double yPos = 11;
  pathLength = bluetoothController.getPathLength();
  int closestIndex = getClosestPathIndex(11, 11, 0);
  int goalIndex = getGoalPointIndex(11, 11, closestIndex);
  // Debug
  Serial.print("closest: "); Serial.println(closestIndex);
  Serial.print("goal: "); Serial.println(goalIndex);
}

void setup() {
  // BEGIN COMMUNICATION
  Serial.begin(BAUD_RATE);
  bluetoothSerial.begin(BAUD_RATE);

  // START THE PID
  pid.SetSampleTime(PID_SAMPLE_TIME);
  pid.SetOutputLimits(-PID_RANGE, PID_RANGE);
  pid.SetMode(AUTOMATIC);

  // SET MOTORS TO MOVE FORWARD
  motorSetForward();

  // ATTACH INTERRUPTS FOR WHEEL MOVEMENTS
  attachInterrupts();

  // DELAY JUST IN CASE BLUETOOTH MODULE NEEDS TIME TO "GET READY"
  delay(100);

  Serial.println("SETUP COMPLETE");
}

void loop() {
  // CHECK FOR INCOMING BLUETOOTH SIGNAL
  bluetoothController.checkBluetooth();


  // IF ROBOT IS CURRENTLY TRAVERSING PATH
  if (isTraversingPath) {
    doDeadReckoning();
    doPurePursuit();
    sendLocationData();
    moveWheels(70, output);
  }

  // IF THE PATH DATA IS AVAILABLE AND ROBOT IS NOT TRAVERSING PATH
  else if (bluetoothController.isPathAvailable()) {
    Serial.println("PATH AVAILABLE");
    // Store path
    pathLength = bluetoothController.getPathLength();
    printPath();

    // Set starting position to first path location point with robot pointing towards the next point.
    double x0 = bluetoothController.getPathX(0);
    double y0 = bluetoothController.getPathY(0);
    double x1 = bluetoothController.getPathX(1);
    double y1 = bluetoothController.getPathY(1);
    double theta = atan2(y1 - y0, x1 - x0);
    deadReckoner.setX(x0);
    deadReckoner.setY(y0);
    deadReckoner.setTheta(theta);

    x0 = deadReckoner.getX();
    y0 = deadReckoner.getY();

    // Debug
    Serial.print("301\tx0: "); Serial.print(x0);
    Serial.print("\ty0: "); Serial.print(y0);
    Serial.print("\tx1: "); Serial.print(x1);
    Serial.print("\ty1: "); Serial.print(y1);

    isTraversingPath = true;
  }

  // STAY STILL
  else {
    moveWheels(0, 0);

    // Debug
    // Serial.println("NOT MOVING");
  }

  /* input = -20;
  moveWheels(80, output);
  doDeadReckoning();
  sendLocationData();
  Serial.println(output); */

  // RUN THE PID ALGORITHM TO UPDATE THE OUTPUT PARAMETER 
  pid.Compute();
}
