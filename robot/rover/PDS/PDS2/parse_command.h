
/*
 * INCOMPLETE!!!!!!!!!!!!!
 */



void parseCommand() 
{
    char *token = strtok(BUFFER, " ");
  
    while (token != NULL) 
    {
        if (*token == 'S') 
        {   //disable all motors
            // disable_multisense();
            disable_motor();
            Serial.println("Command: PDS disabling the motor!");
            break;
        } 
        
        else if (*token == 'A') 
        { //enable all motors
            // enable_multisense();
            enable_motor();
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
                
                switch (fanNum) 
                {
                    case 1:
                        analogWrite(Fan_A_Pin, fanMode);
                        break;
                    case 2:
                        analogWrite(Fan_B_Pin, fanMode);
                        break;
                }
                
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
} // end of parseCommand
