
/*
 * Read Temperature Values
 */



// Constants
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


void mux_settings()
{
    // Prepare address pins for output
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    
    pinMode(MUX_output, INPUT);  
}

float temp_read(int adrs0, int adrs1, int adrs2)
{
    int MUX_OUT = 0;

    float R1 = 10000;
    float log_Rntc, Rntc, T = 0;  
  
    //Write address to mux
    digitalWrite(MUX_S0, adrs0);
    digitalWrite(MUX_S1, adrs1);
    digitalWrite(MUX_S2, adrs2);   

    MUX_OUT = analogRead(MUX_output);

    Rntc = R1 * (1023.0 / (float)MUX_OUT - 1.0);
    log_Rntc = log(Rntc);
    T = (1.0 / (c1 + c2*log_Rntc + c3*log_Rntc*log_Rntc*log_Rntc));
    T = T - 273.15;

    return T;
}
