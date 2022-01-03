#ifndef Vsense
#define Vsense

//assumes Serial.begin() already in main code
//Will print a battery voltage to the screen
void vbatt_read() {
  float vsense = analogRead(V_SENSE);
  // convert to 3.3V reference from analog values (3.3/1023=0.003225806)
  vsense = vsense * 0.003225806;

  // voltage divider backwards (vsense *(r1+r2)/r2
  // = vsense * (10k+2k)/2k = vsense * 6 )
  float vbatt = vsense * 6.0; 

  //UART_PORT.print("ARM battery voltage: ");
  //UART_PORT.println(vbatt);

  if (vbatt < 12.0) {
    //UART_PORT.println("ASTRO WARNING! BATTERY VOLTAGE IS LOW! DISCONNECT IMMEDIATELY");
  }
  else if (vbatt > 16.8) {
    //UART_PORT.println("ASTRO WARNING! BATTERY VOLTAGE IS HIGH! DISCONNECT IMMEDIATELY!");
  }
};

#endif
