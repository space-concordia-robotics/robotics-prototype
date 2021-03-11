
#include <Arduino.h>
//#include <Stepper.h> // Stepper Motor library (Won't be used in future application)
/*#include "variables.h"
#include "sensor_read.h"*/
#include "load_setup.h"
//#include "parse_command.h"
#include "temp_read.h"

//Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() 
{
    Serial.begin(SERIAL_BAUD_RATE);  
    pinMode(13, OUTPUT);
    /*mux_settings();
    load_settings();*/
}

void loop() 
{
    // listen to the serial port and parse an incoming message
//    if (Serial.available()) 
//    {
//        byte num = Serial.readBytesUntil('\n', BUFFER, CHAR_BUFF_SIZE); // Reads the message until it reaches the character '\n'
//        if (num > 0) 
//        {
//            Serial.println(BUFFER); // Prints the received message
//            parseCommand(); // Check the command
//            memset(BUFFER, 0, CHAR_BUFF_SIZE); // Copies 0 to the variable BUFFER
//        } 
//        else 
//        {
//            Serial.println("Command error: PDS received empty message");
//        }
//    }

    Setup mSense;
    mSense.enable_device("Multisense is enabled", mSense.SEn);    
    float load_value = mSense.load_voltage();
    Serial.print("Load Value after conversion: ");
    Serial.print(load_value); 
    Serial.println(" V");

    if (load_value > 5)
    {
        Serial.print("Faulty Condition");
        //1. Interpret the load voltage
        //2. Decide what to do after figuring out the motors' conditions
    }
    

    TempSens temperature;
    // Read Temperature Value
    float Temp1 = temperature.temp_read(0, 0, 0);
  
    Serial.print("Temperature: "); 
    Serial.print(Temp1);
    Serial.println(" C"); 

    if (Temp1 > 35) // what temperature may be dangerous?
    {
        Serial.print("ERROR! Temperature is too high!");
    }    
//    delay(1000);/

    
}
