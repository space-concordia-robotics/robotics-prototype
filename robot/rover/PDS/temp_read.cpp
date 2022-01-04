#include "temp_read.h"
#include <Arduino.h>


TempSens::TempSens(int MUX_S2, int MUX_S1, int MUX_S0, int MUX_output)  //constructor is defined with pins connections
 {
    MUX_0=MUX_S0;
    MUX_1=MUX_S1;
    MUX_2=MUX_S2;
    MUX_O=MUX_output;
    pinMode(MUX_0,OUTPUT);
    pinMode(MUX_1,OUTPUT);
    pinMode(MUX_2,OUTPUT);
    pinMode(MUX_O,INPUT);
 }

void TempSens::select_channel(int ch) //ch 0 to 7
{
   digitalWrite(MUX_2,(ch&7)>>2);
   digitalWrite(MUX_1,(ch&3)>>1);
   digitalWrite(MUX_0,ch&1);
}

float TempSens::read_value(int channel)
{
  select_channel(channel); //select the channel
  MUX_OUT=analogRead(MUX_O);          
  if (channel!=1)
    {
       
       Rntc = R1 * (1023.0 / MUX_OUT - 1.0);
       log_Rntc = log(Rntc);
       T = (1.0 / (c1 + c2*log_Rntc + c3*log_Rntc*log_Rntc*log_Rntc));
       T = T - 273.15;
       if (channel<5)
       {
       Serial.print("TempRead"); 
       Serial.print(channel);
       Serial.print(" ");
       Serial.print(T);
       Serial.println(" C"); 
       return T;
       }
       else
       {
       Serial.print("Board_temp_read"); 
       Serial.print(channel);
       Serial.print(" ");
       Serial.print(T);
       Serial.println(" C"); 
       return T;
       }
        
        
        }
       

 else  //channel 1 is connected to sense Battery voltage

    {
       Vbat=MUX_OUT*11/521;
       Serial.print("Battery Voltage "); 
       Serial.print(Vbat);
       Serial.println("V");
       return Vbat;
    }
}
