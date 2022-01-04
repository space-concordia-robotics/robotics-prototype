#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>


class ParseCommand : public Setup
{
    public:
    
    void command()
    {
        char *token = strtok(BUFFER, " ");
      
        while (token != NULL) 
        {
            if (*token == 'S') 
            {   //disable all motors
                disable_all_motors();
                Serial.println("Command: PDS disabling the motor!");
                break;
            } 
            
            else if (*token == 'A') 
            { //enable all motors
                enable_all_motors();
                Serial.println("Command: PDS enabling the motor!");
                break;
            }
          
            else if (*token == 'F') 
            {
                token = strtok(NULL, " "); //find the next token
                int fanNum = (int)*token - 48;
                
                if (fanNum == 1 || fanNum == 2) 
                {
                    token = strtok(NULL, " "); //find the next token
                    int fanMode = (int)*token - 48;
                    
                    //either fans are on or off, no speed control
                    
                    Serial.println("Command: PDS changing fan mode");
                } 
                break;
            }
            
            else 
            {
                Serial.println("Command error: PDS invalid!"); 
                break;
            }
            
        } // end of while loop
    }
}; // end of parseCommand

#endif
