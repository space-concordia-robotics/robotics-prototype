#ifndef TEMP_H
#define TEMP_H

#include "variables.h"

/*
 * Read Temperature Values
 */

class TempSens
{
    // MUX pins
    int MUX_S0 = 2, MUX_S1 = 3, MUX_S2 = 4;
    int MUX_output = 14;
    // Constants
    const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    
    public:
    
        TempSens(int MUX_S0=2, int MUX_S1=3, int MUX_S2=4, int MUX_output=14);
        float temp_read(int adrs0, int adrs1, int adrs2);    
};

#endif
