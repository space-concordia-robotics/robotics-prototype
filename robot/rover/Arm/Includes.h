/* still in idea phase */
#define DEVEL_MODE_1 1 //!< serial communication over USB, everything unlocked
//#define DEVEL_MODE_2 2 //!< serial communication over UART1, everything unlocked
//#define DEBUG_MODE 3 //!< ROSserial communication over UART1, everything unlocked
//#define USER_MODE 4 //!< ROSserial communication over UART1, functionality restricted
//#define ENABLE_ROS 5 //!< if testing on a computer without ROSserial, comment this to stop errors from rosserial not being installed. obviously you can't use ROSserial if that's the case

// debug statements shouldn't be sent if ros is working
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#define DEBUG_MAIN 10 //!< debug messages during main loop
//#define DEBUG_PARSING 11 //!< debug messages during parsing function
//#define DEBUG_VERIFYING 12 //!< debug messages during verification function
//#define DEBUG_ENCODERS 13 //!< debug messages during encoder interrupts
//#define DEBUG_PID 14 //!< debug messages during pid loop calculations
//#define DEBUG_SWITCHES 15 //!< debug messages during limit switch interrupts
#define DEBUG_HOMING 16 //!< debug messages during homing sequence
//#define DEBUG_DC_TIMER 17 //!< debug messages during dc timer interrupts
//#define DEBUG_SERVO_TIMER 18 //!< debug messages during servo timer interrupts
//#define DEBUG_STEPPER_4_TIMER 20 //!< debug messages during stepper 4 timer interrupts
#endif

/*
  choosing serial vs serial1 should be compile-time: when it's plugged into the pcb,
  the usb port is off-limits as it would cause a short-circuit. Thus only Serial1
  should work.
*/
// serial communication over usb with pc, teensy not connected to odroid

#if defined(DEVEL_MODE_1)
#define UART_PORT Serial
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
#include <Servo.h>

#ifdef ENABLE_ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#endif

#include "PinSetup.h"
#include "Parser.h"
#include "Vsense.h"
// #include "Notes.h" // holds todo info
// #include "Ideas.h" // holds bits of code that haven't been implemented
#include "PidController.h"
#include "RobotMotor.h"
#include "DcMotor.h"
#include "ServoMotor.h"
/* interrupt priorities */
#define LIMIT_SWITCH_NVIC_PRIORITY 100 //!< limit switch interrupt priority is highest
#define ENCODER_NVIC_PRIORITY LIMIT_SWITCH_NVIC_PRIORITY + 4 //!< encoder interrupt priority is second highest
#define MOTOR_NVIC_PRIORITY ENCODER_NVIC_PRIORITY + 4 //!< motor timer interrupt priority is third highest
/* heartbeat */
#define MAX_GOOD_BLINKS 2
#define MAX_BAD_BLINKS 10
#define GOOD_BLINK_PERIOD 250
#define BAD_BLINK_PERIOD 70
#define HEARTBEAT_PERIOD 1000
/* serial */
#define BAUD_RATE 115200 //!< serial bit rate
#define SERIAL_PRINT_INTERVAL 50 //!< how often should teensy send angle data
#define SERIAL_READ_TIMEOUT 20 //!< how often should the serial port be read
#define BUFFER_SIZE 100 //!< size of the buffer for the serial commands
