void reading() {
  Serial.println("Starting reading");
   /*
  * phi_1 HIGH, phi_2 LOW
  * SH HIGH
  * SH LOW
  * phi_1 LOW, phi_2 HIGH  
  */
  PORTB |= 0x02;  //PHI_1 HIGH
  PORTB &= ~(0x04); //PHI_2 LOW

  PORTB |= 0x01;  //SH HIGH
  //DELAY
//  for(int i = 0 ; i < 1000 ; i++){}
  delayMicroseconds(4);
  PORTB &= ~(0x01); //SH LOW

//  PORTB &= ~(0x02); //PHI_1 LOW
//  PORTB |= 0x04;  //PHI_2 HIGH
delayMicroseconds(2);

  for(int i = 0 ; i < 2090 ; i++){
    //phi_1 LOW
    //phi_2 HIGH
    PORTB &= ~(0x02); //PHI_1 LOW
    PORTB |= 0x04;  //PHI_2 HIGH

  
    //RS HIGH
    PORTB |= 0x08; //RS HIGH
  
    //RS LOW
    PORTB &= ~(0x08); //RS LOW
    
    //CP HIGH
    PORTB |= 0x10;  //CP HIGH
    
    //CP LOW
    PORTB &= ~(0x10); //CP LOW
    
    //phi_1 HIGH
    //phi_2 LOW
    PORTB |= 0x02;  //PHI_1 HIGH
    PORTB &= ~(0x04); //PHI_2 LOW
    
  } 
  Serial.println("Done reading");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  DDRB |= 0xFF;

  //all LOW, phi2 HIGH
  PORTB |= 0x04;
  PORTB &= ~(0x1B) ;
  
}

/*
 * B0 i.e D8  = SH    i.e pin 22 ; SH    --> shift gate
 * B1 i.e D9  = phi_1 i.e pin 5  ; phi_1 --> transfer clock (phase 1)
 * B2 i.e D10 = phi_2 i.e pin 6  ; phi_2 --> transfer clock (phase 2)
 * B3 i.e D11 = RS    i.e pin 18 ; RS    --> reset gate
 * B4 i.e D12 = CP    i.e pin 21 ; CP    --> clamp gate
 */



void loop() {

  if (Serial.available() > 0) {
    char byteBuffer = Serial.read();

    if (byteBuffer == 'r') {
      Serial.print("Received: ");
      Serial.print(byteBuffer);
      Serial.println();
      reading();
    }
  }
  
}
