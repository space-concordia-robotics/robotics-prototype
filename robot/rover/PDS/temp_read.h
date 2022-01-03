#include <Arduino.h>
#ifndef TEMP_H
#define TEMP_H
/*
 * Read Temperature Values
 */

class TempSens
{
        // Constants
    const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    float R1 = 10000,log_Rntc, Rntc, T = 0,MUX_OUT,Vbat;  
    int MUX_0,MUX_1,MUX_2,MUX_O;

  
    public:
    
        TempSens(int MUX_S2, int MUX_S1, int MUX_S0, int MUX_output);  //constructor is defined with pins connections
        void select_channel(int ch); //ch 0 to 7
        float read_value(int channel);                                   
 };

#endif
