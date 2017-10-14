/*------------------------------------------------------------------------------------------------------------*/
/*                                            Libraries & Classes                                             */
/*------------------------------------------------------------------------------------------------------------*/
#include<Servo.h> //Servo library
#include "IRremote.h" //InfraRed remote library, also we need to remove <RobotIRremote> if installed
#include <LiquidCrystal.h> //LCD library

/*------------------------------------------------------------------------------------------------------------*/
/*                                                  Variables                                                 */
/*------------------------------------------------------------------------------------------------------------*/
//Declaring the pins for the LCD
LiquidCrystal lcd(7,8,9,10,11,12);

//Declaring the pin for the remote
const int ReceiverPin = 6; //This will be the signal pin for the receiver module

//Here are some variables for the remote
IRrecv irrecv(ReceiverPin); //This create a Receiver object
decode_results results; //This will store and code received

//Declaring the servo's Pins
const int APin = 2;
const int RPin = 3;
const int ZPin = 4;
const int CPin = 5;//For the Claw ServoMotor

//Creating the servos
Servo AServo; // ANGLE Servo
Servo RServo; // RADIAL Servo
Servo ZServo; // HEIGHT Servo
Servo CServo; // CLAW Servo

//Declaring constants for the coordinate systems
//I will use Polar coordinates to control the arm
int APos = 90, AMin = 0 , AMax = 180;
int RPos = 60, RMin = 60, RMax = 180;
int ZPos = 90, ZMin = 0, ZMax = 120;
int CPos = 90, CMin = 70 , CMax = 130;
bool CState = 0;

//Variables in arrays for the memorized positions
int APosEQ[10];
int RPosEQ[10];
int ZPosEQ[10];
int CPosEQ[10];

//Variables use throught out the program
char Last =  'O'; //The last move that has been done
int wait = 0; //This control the delays throught the loop
int Step = 2; //Angle difference for each time a button is pressed
bool EQ = 0; //This is the variable for GOTO VS AddPosition


/*------------------------------------------------------------------------------------------------------------*/
/*                                                  FUNCTIONS                                                 */
/*------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------ADD POSITION------------------------------------------------*/
void AddPosition(int i){
  Last = 'O';
  if (EQ == 1){
    APosEQ[i] = APos;
    RPosEQ[i] = RPos;
    ZPosEQ[i] = ZPos;
    CPosEQ[i] = CPos;
    EQ =0;
  }
  else {
    GOTO(APosEQ[i],RPosEQ[i],ZPosEQ[i],CPosEQ[i]);
  }
}

/*-------------------------------------------------GOTO---------------------------------------------------------*/
void GOTO(int A,int R,int Z, int C){
  APos = A;
  RPos = R;
  ZPos = Z;
  CPos = C;

  AServo.write(APos);
  RServo.write(RPos);
  ZServo.write(ZPos);
  CServo.write(CPos);
}

/*------------------------------------------------HOME-----------------------------------------------------------*/
void Home(){ //Will reset the robot to its "Home" position when called
  APos = 90;
  RPos = 60;
  ZPos = 90;
  CPos = CMin;

  AServo.write(APos);
  RServo.write(RPos);
  ZServo.write(ZPos);
  CServo.write(CMin);

  Last = 'H';
}

/*----------------------------------------------UPDATE LCD--------------------------------------------------------*/
void UpdateLCD() { //This will do as it says
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("|A| |R| |Z| |C|");
  lcd.setCursor(0,1);
  lcd.print(APos);
  if (APos <100)
    lcd.print(" ");
  if (APos <10)
    lcd.print(" ");
  lcd.print(" ");
  lcd.print(RPos);
  if (RPos <100)
    lcd.print(" ");
  if (RPos <10)
    lcd.print(" ");
  lcd.print(" ");
  lcd.print(ZPos);
  if (ZPos <100)
    lcd.print(" ");
  if (ZPos <10)
    lcd.print(" ");
  lcd.print(" ");
  lcd.print(CPos);
  if(CPos<100)
    lcd.print(" ");
  if(CPos<10)
    lcd.print(" ");
  lcd.print(EQ);
}

/*-----------------------------------------------REMOTE MODE------------------------------------------------------*/
void RemoteMode() { //This function controls the robot arm from the IR remote

  switch(results.value)  { // The values of the cases where obtain on the internet.

    case 0xFFA25D: { //POWER //This will turn on/off the remote control mode
      Serial.println("POWER");
      break;
    }
    case 0xFFE21D: { //FUNC/STOP //This will  move the arm to its home position
      Serial.println("FUNC/STOP");
      Home();
      break;
    }
    case 0xFF629D: { //VOL+ //This will move the arm forward
      Serial.println("VOL+");
      if (RPos < RMax){
        Last = 'F';
        RPos+=Step;
        RServo.write(RPos);
        delay(wait);
      }
      break;
    }
    case 0xFF22DD: { //FAST BACK |<< //This will move the arm to the left
      Serial.println("FAST BACK");
      if (APos > AMin){
        Last = 'L';
        APos-=Step;
        AServo.write(APos);
        delay(wait);
      }
      break;
    }
    case 0xFF02FD: { //PLAY/PAUSE //This will be use to Open/Close the claw
      Serial.println("PAUSE");
      Serial.print("CState =");
      Serial.println(CState);
      Last = 'C';
      if (CState == 0) {//Open Claw
        CPos = CMax;
        CServo.write(CPos);
        CState = 1;
      }
      else if (CState == 1) {//close Claw
        CPos = CMin;
        CServo.write(CPos);
        CState = false;
      }
      break;
    }
    case 0xFFC23D: { //FAST FORWARD >>| //This will move the arm to the right
      Serial.println("FAST FORWARD");
      if (APos < AMax){
        Last = 'R';
        APos+=Step;
        AServo.write(APos);
        delay(wait);
      }
      break;
    }
    case 0xFFE01F: { //DOWN //This will move the arm down
      Serial.println("DOWN");
      if (ZPos > ZMin){
        Last = 'D';
        ZPos-=Step;
        ZServo.write(ZPos);
        delay(wait);
      }
      break;
    }
    case 0xFFA857: { //VOL- //This will move the arm back
      Serial.println("VOL-");
      if (RPos > RMin){
        Last = 'B';
        RPos-=Step;
        RServo.write(RPos);
        delay(wait);
      }
      break;
    }
    case 0xFF906F: { //UP //This will move the arm up
      Serial.println("UP");
      if (ZPos < ZMax){
        Last = 'U';
        ZPos+=Step;
        ZServo.write(ZPos);
        delay(wait);
      }
      break;
    }
    case 0xFF9867: { //EQ // This decide whether to register the new coordinate or to simply goto the position
      Serial.println("EQ");
      Last = 'H';
      if (EQ ==0){
        EQ = 1;
      }
      else if (EQ == 1) {
        EQ = 0;
      }
      break;
    }
    case 0xFFB04F: { //ST/REPT //This does nothing.. for now ;)
      Serial.println("ST/REPT");
      break;
    }
    case 0xFF6897: { //0 //Registed Position #0
      Serial.println("0");
      AddPosition(0);
      break;
    }
    case 0xFF30CF: { //1//Registed Position #1
      Serial.println("1");
      AddPosition(1);
      break;
    }
    case 0xFF18E7: { //2 //Registed Position #2
      Serial.println("2");
      AddPosition(2);
      break;
    }
    case 0xFF7A85: { //3 //Registed Position #3
      Serial.println("3");
      AddPosition(3);
      break;
    }
    case 0xFF10EF: { //4 //Registed Position #4
      Serial.println("4");
      AddPosition(4);
      break;
    }
    case 0xFF38C7: { //5 //Registed Position #5
      Serial.println("5");
      AddPosition(5);
      break;
    }
    case 0xFF5AA5: { //6 //Registed Position #6
      Serial.println("6");
      AddPosition(6);
      break;
    }
    case 0xFF42BD: { //7 //Registed Position #7
      Serial.println("7");
      AddPosition(7);
      break;
    }
    case 0xFF4AB5: { //8 //Registed Position #8
      Serial.println("8");
      AddPosition(8);
      break;
    }
    case 0xFF52AD: { //9 //Registed Position #9
      Serial.println("9");
      AddPosition(9);
      break;
    }
    case 0xFFFFFFFF: { //This is toggle when a button is HELD, so it makes the last move it made
      Serial.println(" REPEAT");
      if (Last == 'R' && APos < AMax){ // If last was Right
         AServo.write(APos+5);
         APos+=Step;
         delay(wait);
      }
      if (Last == 'L' && APos > AMin){ // If last was Left
         AServo.write(APos);
         APos-=Step;
         delay(wait);
      }
      if (Last == 'B' && RPos > RMin){ // If last was Back
        RPos-=Step;
        RServo.write(RPos);
        delay(wait);
      }
      if (Last == 'F' && RPos < RMax){ // If last was Forward
        RPos+=Step;
        RServo.write(RPos);
        delay(wait);
      }
      if (Last == 'U' && ZPos < ZMax){ // If last was Up
        ZPos+=Step;
        ZServo.write(ZPos);
        delay(wait);
      }
      if (Last == 'D' && ZPos > ZMin){ // If last was Down
        ZPos-=Step;
        ZServo.write(ZPos);
        delay(wait);
      }
      break;
    }
    default: { //This is when the IR sensor doen't know what was the button pressed
      Serial.println(" other button   ");
      Last = 'O'; //Puttin last = to O as an "Do Nothing"
    }

  }//End Case

} //End of RemoteMode

/*----------------------------------SETUP-------------------------------------------*/
void setup() {
  Serial.begin(9600);//Specifying the baudrate

  //Attaching the servos to their Pin
  AServo.attach(APin);
  RServo.attach(RPin);
  ZServo.attach(ZPin);
  CServo.attach(CPin);

  //I will initialize the Robot position to "home"
  Home ();

  //Enable the IR Receiver
  irrecv.enableIRIn();

  //Initializing the values in the arrays to home for the registed position
  for(int i=0;i<10;i++){
    APosEQ[i] = 90;
    RPosEQ[i] = 60;
    ZPosEQ[i] = 90;
    CPosEQ[i] = CMin;
    Serial.println(APosEQ[i]);
    Serial.println(RPosEQ[i]);
    Serial.println(ZPosEQ[i]);
    Serial.println(CPosEQ[i]);
  }

  //Initializing the LCD Screen
  lcd.begin(16,2);
  lcd.print("  Ben St-Pierre");
  lcd.setCursor(0,1);
  lcd.print("RobotArm Program");
  delay(3000);
  lcd.clear();

} //End of Setup

/*-------------------------------------LOOP-------------------------------------------*/
void loop() {

  if(irrecv.decode(&results)){ //If the sensor detected any result

    RemoteMode(); //Switch the result to know what to do

    //For debug only
    Serial.print("Last = ");
    Serial.println(Last);
    Serial.print("APos = ");
    Serial.println(APos);
    Serial.print("RPos = ");
    Serial.println(RPos);
    Serial.print("ZPos = ");
    Serial.println(ZPos);
    Serial.print("CPos = ");
    Serial.println(CPos);
    Serial.print("Hex Code = ");
    Serial.println(results.value, (HEX));
    Serial.print("EQ = ");
    Serial.println(EQ);
    Serial.println();

    irrecv.resume(); //Get ready for the next result

  }//End If

  UpdateLCD();

  delay(25);

} // End of Loop

//The goal of this program is to be able to control my robot arm with a IR remote control.
//I would like that the robot is in sleep mode by default and when it receives an signal
//it goes into remote mode. Then when a button is pressed, the robot moves accordingly.
