//#ifndef
//#define

/*
 * Pins and Variable Definitions
 */


#define CHAR_BUFF_SIZE 150
#define SERIAL_BAUD_RATE 9600

char BUFFER[CHAR_BUFF_SIZE]; //!< used for sending/receiving/reading messages


// Number of steps per rotation for Stepper Motor (This will be removed for our application)
// const int stepsPerRevolution = 2048;

//#endif/



/*
 * 1. vn5050 (7050)  -->> how to simulate faulty condition? what to do if a faulty condition happens?
 * 2. parseCommand  -->> we have a gui, we should make it work for the system
 * 3. what to do with the temps?
 */
