#include <Arduino.h>
#include "PinSetup.h"

void setupPins(void){
	//pinMode(ENCODER_A, INPUT);
	//pinMode(ENCODER_B, INPUT);
	//pinMode(interruptPin, INPUT_PULLUP);
	/*which do I use?*/
	pinMode(HEARTBEAT_PIN, OUTPUT);

	pinMode(M1_ENABLE_PIN, OUTPUT);
	pinMode(M1_DIR_PIN, OUTPUT);
	pinMode(M1_STEP_PIN, OUTPUT);
	/* #define M1_ENCODER_A        7
	#define M1_ENCODER_B        8
	#define M1_LIMIT_SW_CW      9
	#define M1_LIMIT_SW_CCW    10 */

	pinMode(M3_ENABLE_PIN, OUTPUT);
	pinMode(M3_DIR_PIN, OUTPUT);
	pinMode(M3_STEP_PIN, OUTPUT);
	/* #define M3_ENCODER_A       20
	#define M3_ENCODER_B       21
	#define M3_LIMIT_SW_FLEX   22
	#define M3_LIMIT_SW_EXTEND 23 */

	pinMode(M4_ENABLE_PIN, OUTPUT);
	pinMode(M4_DIR_PIN, OUTPUT);
	pinMode(M4_STEP_PIN, OUTPUT);
	/* #define M4_ENCODER_A       24
	#define M4_ENCODER_B       14
	#define M4_LIMIT_SW_FLEX   15
	#define M4_LIMIT_SW_EXTEND 16 */

	// DC motor

	pinMode(M2_PWM_PIN, OUTPUT);
	//M2_UART_TX         32
	//M2_UART_RX         31
	/* #define M2_ENCODER_A       26
	#define M2_ENCODER_B       27
	#define M2_LIMIT_SW_FLEX   28
	#define M2_LIMIT_SW_EXTEND 29 */

	// servos

	pinMode(M5_PWM_PIN, OUTPUT);
	//#define M5_ENCODER_A       34
	//#define M5_ENCODER_B       33

	pinMode(M6_PWM_PIN, OUTPUT);
	//#define M6_ENCODER_A       37
	//#define M6_ENCODER_B       38
}
