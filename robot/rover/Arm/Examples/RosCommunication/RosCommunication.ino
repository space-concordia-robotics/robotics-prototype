/*
This sketch is to test the communication between a Teensy and another device
through ROS. It allows for communication over any serial port, though both
usb and hardware serial remain to be tested. The appropriate parameter must
be selected in the first section of this code. Currently this code publishes 6
angle values to their respective topics "mX_joint_state" with frames "mX_angle",
where X is the motor number, using JointState messages. It also
subscribes to the arm_command topic using String messages to listen in for
certain commands like "stop" and "motor 3 angle -42.6". Furthermore, the sketch
is supposed to log 3 types of information: debug, info, and error. Debug is for
noncritical messages that prove the code is working as expected. Info is currently
for sending clear responses to confirm that a certain behaviour has succeeded.
Error is currently for reporting when a command has failed. Finally, a service
may be implemented which listens for "ping" and reponds with "pong". Currently
the "pong" response is logged as info.
*/

/* still in idea phase */
//#define DEVEL_MODE_1 1 // sends messages with Serial, everything unlocked
//#define DEVEL_MODE_2 2 // sends messages with Serial1, everything unlocked
#define DEBUG_MODE 3 // sends messages with ROSserial, everything unlocked
// #define USER_MODE 4 // sends messages with ROSserial, functionality restricted

// debug statements shouldn't be sent if ros is working
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#define DEBUG_MAIN 10 // debug messages during main loop
//#define DEBUG_PARSING 11 // debug messages during parsing function
//#define DEBUG_VERIFYING 12 // debug messages during verification function
#endif

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
// serial communication over usb with pc, teensy not connected to odroid

#if defined(DEVEL_MODE_1)
#define UART_PORT Serial
// serial communication over uart with odroid, teensy plugged into pcb and odroid
#elif defined(DEVEL_MODE_2)
#define UART_PORT Serial1
#endif
#if defined(DEBUG_MODE) || defined(USER_MODE)
#define USE_TEENSY_HW_SERIAL 0 // this will make ArduinoHardware.h use hardware serial instead of usb serial
#endif
/*
  choosing serial1 vs rosserial could be compile-time, since serial1 is only really useful
  for debugging and won't be used when the rover is in action. however, a runtime option
  could be useful as in both cases the teensy is communicating solely with the odroid.
  it might be desirable to switch between modes without recompiling.
  finally, unlocking extra options should be runtime as it should be easily accessible.
*/
// includes must come after the above UART_PORT definition as it's used in other files.
// perhaps it should be placed in pinsetup.h (which has to be renamed anyway)...
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
// homemade includes
#include "Parser.h"

// normally this goes in pinSetup.h but the rest of that stuff isn't useful so i got rid of it
#define NUM_MOTORS 6 // used in parsing for commands for multiple motors

/* comms */
#define BAUD_RATE 115200 // bit rate over serial communications (currently not for ROS)
#define SERIAL_PRINT_INTERVAL 1000 // how often should the teensy send angle data
#define SERIAL_READ_TIMEOUT 50 // how often should the serial port be read (currently not for ROS)
#define BUFFER_SIZE 100 // size of the buffer for the serial commands (not for ROS)

char serialBuffer[BUFFER_SIZE]; // serial buffer used for early- and mid-stage tesing without ROSserial
String messageBar = "======================================================="; // for clarity of print statements

/*
  info from parsing functionality is packaged and given to motor control functionality.
  many of these are set to 0 so that the message can reset, thus making sure that
  the code later on doesn't inadvertently make a motor move when it wasn't supposed to
*/
// kinda weird that this comes out of nowhere... maybe should be Parser.commandInfo or something.
// or define commandInfo here instead of in Parser
commandInfo emptyMotorCommand; // emptyMotorCommand is used to reset the struct when the loop restarts
commandInfo motorCommand; // motorCommand holds the actual command of the current loop
Parser Parser; // object which parses serialBuffer and verifies commandInfo structs

bool msgReceived = false; // for ros only, since Serial has Serial.available()
bool msgIsValid = false; // if this is true, then the Teensy will actually try doing something

// develmode1 actually isn't for ros... i will have to change things if i want ros over usb
#ifdef DEVEL_MODE_1 // using the USB port
//ros::NodeHandle nh;
#elif defined(DEBUG_MODE) || defined(USER_MODE) // using hardware serial (Serial1 in this case)
// the following commented block allows you to choose a hardware serial port that isn't Serial1
/*
  class NewHardware : public ArduinoHardware {
  public:
  long baud = 57600;
  NewHardware():ArduinoHardware(&Serial1, baud){}; // place the serial port of your choosing (1 to 6)
  };

  ros::NodeHandle_<NewHardware> nh;
*/
// otherwise just use this
ros::NodeHandle nh; // this handle is what lets the Teensy work like a ROS node
#endif

// I define this here because that's how the examples do it.
// I think it has to be before the subscriber is instantiated but usually i put function definitions down below. test leaving it as just a declaration?
// I think this is called when you use nh.spinOnce() and a message with the appropriate topic was received
void messageCallback(const std_msgs::String& cmd_message) {
  msgReceived = true;
  int i = 0;
  // place the message into serialBuffer for parsing in setup()
  while (cmd_message.data[i] != '\0') {
    serialBuffer[i] = cmd_message.data[i];
  }
}
// create the subscriber that actually calls the messageCallback function
ros::Subscriber<std_msgs::String> cmdSubscriber("arm_command", &messageCallback);

// not sure what these do yet, probably for URDF or tf stuff
char m1FrameId[] = "/m1_angle"; char m2FrameId[] = "/m2_angle"; char m3FrameId[] = "/m3_angle";
char m4FrameId[] = "/m4_angle"; char m5FrameId[] = "/m5_angle"; char m6FrameId[] = "/m6_angle";

// instantiate a JointState message for each motor joint
sensor_msgs::JointState m1_angle_msg; sensor_msgs::JointState m2_angle_msg;
sensor_msgs::JointState m3_angle_msg; sensor_msgs::JointState m4_angle_msg;
sensor_msgs::JointState m5_angle_msg; sensor_msgs::JointState m6_angle_msg;

// make an array of the messages which helps reduce code size
sensor_msgs::JointState angleMessages[NUM_MOTORS] = {m1_angle_msg, m2_angle_msg, m3_angle_msg, m4_angle_msg, m5_angle_msg, m6_angle_msg};

// create the publishers which send angle data to the appropriate topics for the Odroid to listen to
ros::Publisher pub_m1("m1_joint_state", &m1_angle_msg); ros::Publisher pub_m2("m2_joint_state", &m2_angle_msg);
ros::Publisher pub_m3("m3_joint_state", &m3_angle_msg); ros::Publisher pub_m4("m4_joint_state", &m4_angle_msg);
ros::Publisher pub_m5("m5_joint_state", &m5_angle_msg); ros::Publisher pub_m6("m6_joint_state", &m6_angle_msg);

// or just make one message for all the angles
//std_msgs::String joint_angles_msg;
// and just publish a String and parse it on the odroid
//ros::Publisher joint_angles_publisher("arm_joint_angles", &joint_angles_msg)

// these are a nicer way of timing events than using millis()
elapsedMillis sinceAnglePrint; // how long since last time angle data was sent
/* function declarations */
void printMotorAngles(void); // sends all motor angles over serial

/* Teensy setup */
void setup() {
  pinSetup(); // initializes all the appropriate pins to outputs or interrupt pins etc
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  // set up the serial port
  UART_PORT.begin(BAUD_RATE);
  UART_PORT.setTimeout(SERIAL_READ_TIMEOUT); // checks serial port every 50ms
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  // set up the ROS node stuff
  nh.initNode();
  nh.subscribe(cmdSubscriber);
  nh.advertise(pub_m1);  nh.advertise(pub_m2);  nh.advertise(pub_m3);
  nh.advertise(pub_m4);  nh.advertise(pub_m5);  nh.advertise(pub_m6);
  // set the header, probably for URDF or tf stuff later
  m1_angle_msg.header.frame_id = m1FrameId;  m2_angle_msg.header.frame_id = m2FrameId;
  m3_angle_msg.header.frame_id = m3FrameId;  m4_angle_msg.header.frame_id = m4FrameId;
  m5_angle_msg.header.frame_id = m5FrameId;  m6_angle_msg.header.frame_id = m6FrameId;
  // do I need to set up a header for joint_angles_msg with "arm_joint_angles" topic?
#endif
  // reset the elapsedMillis variables so that they're fresh upon entering the loop()
  sinceAnglePrint = 0;
}

/* main code loop */
void loop() {
  // forget whatever data was put in here during the previous loop because it's a new day
  memset(serialBuffer, 0, BUFFER_SIZE); // empty the buffer
  motorCommand = emptyMotorCommand;
  msgReceived = false;
  msgIsValid = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  if (UART_PORT.available()) { // if a message was sent to the Teensy, the check for messages is implicit in arduino
    UART_PORT.readBytesUntil(10, serialBuffer, BUFFER_SIZE); // read through it until NL
#ifdef DEBUG_MAIN
    UART_PORT.println(messageBar);
    UART_PORT.print("GOT: ");
    UART_PORT.println(serialBuffer); // send back what was received
#endif
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  nh.spinOnce(); // check for new commands, explicit with ROS
  if (msgReceived) { // if a message was sent to the Teensy
    nh.logdebug(serialBuffer); // send back what was received
#endif
    Parser.parseCommand(motorCommand, serialBuffer); // put the stuff from serialBuffer into motorCommand
    msgIsValid = Parser.verifCommand(motorCommand); // check if the command has invalid params
  } // this might cause problems because it closes something that will only exist if the appropriate things are defined
  if (msgIsValid) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#ifdef DEBUG_MAIN
    UART_PORT.println(messageBar);
#endif
#endif
    if (motorCommand.pingCommand) {
      // respond to ping, turn this into a service later on maybe
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("pong");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
      nh.loginfo("pong");
#endif
    }
    // emergency stop takes precedence
    else if (motorCommand.stopAllMotors) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("emergency stop command validated");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
      nh.loginfo("emergency stop command validated");
#endif
    }
    else {
      // stopping a single motor takes precedence
      if (motorCommand.stopSingleMotor) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
        UART_PORT.print("if this code did something, it would stop motor ");
        UART_PORT.println(motorCommand.whichMotor);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
        // this is SUPER DUPER GROSS
        String infoMessage = "stopped motor " + motorCommand.whichMotor;
        char actualMessage[75];
        for (unsigned int i = 0; i < infoMessage.length(); i++) {
          actualMessage[i] = infoMessage[i];
        }
        // if necessary, add '\0' to the i++ element of the array to indicate its end
        nh.loginfo(actualMessage);
#endif
      }
      // make motors move simultaneously
      else if (motorCommand.multiMove) {
        for (int i = 0; i < NUM_MOTORS; i++) {
          if (motorCommand.motorsToMove[i]) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("motor "); UART_PORT.print(i + 1);
            UART_PORT.print(" desired angle (degrees) is: ");
            UART_PORT.println(motorCommand.anglesToReach[i]);
#elif defined(DEBUG_MODE) || defined(USER_MODE)
            // this is SUPER DUPER GROSS
            int tempVal = i + 1; String infoMessage = "motor " + tempVal;
            infoMessage += " desired angle (degrees) is: ";
            infoMessage += motorCommand.anglesToReach[i];
            char actualMessage[75];
            for (unsigned int i = 0; i < infoMessage.length(); i++) {
              actualMessage[i] = infoMessage[i];
            }
            // if necessary, add '\0' to the i++ element of the array to indicate its end
            nh.loginfo(actualMessage);
#endif
          }
          else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
            UART_PORT.print("if this code did something, motor "); UART_PORT.print(i + 1);
            UART_PORT.println(" would not change course");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
            // this is SUPER DUPER GROSS
            int tempVal = i + 1;
            String infoMessage = "if this code did something, motor " + tempVal;
            infoMessage += " would not change course";
            char actualMessage[75];
            for (unsigned int i = 0; i < infoMessage.length(); i++) {
              actualMessage[i] = infoMessage[i];
            }
            // if necessary, add '\0' to the i++ element of the array to indicate its end
            nh.loginfo(actualMessage);
#endif
          }
        }
      }
    }
  }
  else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
    UART_PORT.println("$E,Error: bad motor command");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
    nh.logerror("bad motor command");
#endif
  }
  // every SERIAL_PRINT_INTERVAL milliseconds the Teensy should print all the motor angles
  if (sinceAnglePrint >= SERIAL_PRINT_INTERVAL) {
    printMotorAngles();
    sinceAnglePrint = 0; // reset the timer
  }
}

void printMotorAngles(void) {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
  UART_PORT.println("Motor angles undefined");
#elif defined(DEBUG_MODE) || defined(USER_MODE)
  float angles[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    angles[i] = i+0.7; // generate fictional angle values
    angleMessages[i].position = &(angles[i]); // update the respective message with the respective angle value
  }
  // publish the angle values
  pub_m1.publish(&m1_angle_msg);  pub_m2.publish(&m2_angle_msg);  pub_m3.publish(&m3_angle_msg);
  pub_m4.publish(&m4_angle_msg);  pub_m5.publish(&m5_angle_msg);  pub_m6.publish(&m6_angle_msg);
  // or if I'm publishing the String
  //joint_angles_publisher.publish(&joint_angles_msg);

  // does it cause problems if i spin twice in loop()?
  nh.spinOnce(); // this actually sends the message
#endif
}