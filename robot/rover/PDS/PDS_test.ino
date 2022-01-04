#include "variables.h"
#include "load_setup.h"
#include "parse_command.h"
#include "temp_read.h"

TempSens temperature(8,3,4,A6); // object argument is pin number in order (MUXS2,MUXS1,MUXS0,MUXoutput)
//NOTE: TempSens senses battery voltage at channel 4, conversion is taken care by the header file.

Setup load_operation;
float current,Temp;
void setup() 
{
    Serial.begin(9600);
    load_operation.disable_all_motors();
}
void check_temp();
void check_load_current();

void loop() 
{
    float v,sense;
    /*ParseCommand cmd;
    
    // listen to the serial port and parse an incoming message
    if (Serial.available()) 
    {
        byte num = Serial.readBytesUntil('\n', BUFFER, CHAR_BUFF_SIZE); // Reads the message until it reaches the character '\n'
        if (num > 0) 
        {
            Serial.println(BUFFER); // Prints the received message
            cmd.command(); // Check the command
            memset(BUFFER, 0, CHAR_BUFF_SIZE); // Copies 0 to the variable BUFFER
        } 
        else 
        {
            Serial.println("Command error: PDS received empty message");
        }
    }

   
    */ //temperature.read_value(1);
    
    char r;
    float pre_cur;
    if(Serial.available())
    {
        r=Serial.read();
    }
    if (r=='e'){
        load_operation.enable_all_motors();
    }
    if(r=='d')
    {
        load_operation.disable_all_motors();
    }

    current=load_operation.load_current(5);
    if(current>3)
        load_operation.disable_all_motors();
  

/*
//current= load_operation.load_current(5);
pre_cur= temperature.read_value(1);    

if(current>3)
{
load_operation.disable_all_motors();
Serial.println("Boom");
  */
  
}

void check_load_current()
{
   current= load_operation.load_current(1); //load_current(load_switch number) returns current in Ampere through motor number 
   //add CTA  
}

void check_temp()
{
    Temp = temperature.read_value(0); //read_value(channel) returns temperature in C if channel !=1, for channel=1, it returns battery voltage

    if (Temp > 60) // what temperature may be dangerous?
    {
        Serial.print("Warning! Temperature is too high!");
    }    
}
