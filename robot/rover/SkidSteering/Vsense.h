#ifndef Vsense
#define Vsense

//assumes Serial.begin() already in main code
//Will print a battery voltage to the screen
void vbatt_read() {
  float vsense = analogRead(V_SENSE_PIN);
  vsense = vsense * 0.003225806; //convert to 3.3V reference from analog values (3.3/1023=0.003225806)

  float vbatt = vsense * 6;     //voltage divider backwards (vsense *(r1+r2)/r2 = vsense * (10k+2k)/2k
  // = vsense * 6 )

  PRINT("ASTRO Battery voltage: ");
  PRINT(vbatt);

  if (vbatt < 12) {
    PRINTln("ASTRO WARNING! BATTERY VOLTAGE IS LOW! DISCONNECT IMMEDIATELY");
  }
  else if (vbatt > 16.8) {
    PRINTln("ASTRO WARNING! BATTERY VOLTAGE IS HIGH! DISCONNECT IMMEDIATELY!");
  }
};

#endif
