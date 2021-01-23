
/*
 * Pins and Variable Definitions
 */

 

// MUX pins
#define MUX_S0  2
#define MUX_S1  3
#define MUX_S2  4

#define MUX_output A0

// FANs pins
#define Fans 5 // PB1
#define Fan_B_Pin 6 // PB2/
#define Fan_A_Pin 6 // PB2/

// MOTOR pins are Arduino's digital 8, 9, 10, 11 (This will change for our application)
#define MOTOR_in 7

// VN7050 MultiSense pin
#define MultiSense A1

// VN7050 SEn pin
#define SEn 12



#define CHAR_BUFF_SIZE 150
#define SERIAL_BAUD_RATE 9600



char BUFFER[CHAR_BUFF_SIZE]; //!< used for sending/receiving/reading messages


// Temperature value
float Temp1 = 0;


// Number of steps per rotation for Stepper Motor (This will be removed for our application)
const int stepsPerRevolution = 2048;
