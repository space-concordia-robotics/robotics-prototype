#ifndef COMMANDS_H
#define COMMANDS_H

class Commands{
  public:
    String ledManualCtrl_cmd = "indicator-manual\n";
    String ledAutoCtrl_cmd = "indicator-autonomous\n";
    String ledEureka_cmd = "indicator-eureka\n";
    String ledOn_cmd = "indicator-on\n";
    String ledOff_cmd = "indicator-off\n";
    int indicatorRgbArray[3] = {0,0,0};
    bool isLedOn = false;
    void ledManualCtrl(void);
    void ledAutoCtrl(void);
    void ledEureka(void);
    void handler(String);
    void ledOff(void);
    void ledOn(void);
};

void Commands::handler(String cmd){
  if(cmd == ledManualCtrl_cmd){
    ledManualCtrl();
  }
  else if(cmd == ledAutoCtrl_cmd){
    ledAutoCtrl();
  }
  else if(cmd == ledEureka_cmd){
    ledEureka();
  }
  else if(cmd == ledOn_cmd){
    if(isLedOn == false){
        ledOn();
    }
  }
  else if(cmd == ledOff_cmd){
    if(isLedOn == true){
      ledOff();
    }
  }
}

void Commands::ledOn(void){
    isLedOn = true;
    Serial.println("on");
}

void Commands::ledOff(void){
    isLedOn = false;
}
void Commands::ledManualCtrl(void){
    Serial.println("manual");
    indicatorRgbArray[0] = 0;
    indicatorRgbArray[1] = 0;
    indicatorRgbArray[2] = 255;
}

void Commands::ledAutoCtrl(void){
    indicatorRgbArray[0] = 255;
    indicatorRgbArray[1] = 0;
    indicatorRgbArray[2] = 0;
}

void Commands::ledEureka(void){
    indicatorRgbArray[0] = 0;
    indicatorRgbArray[1] = 255;
    indicatorRgbArray[2] = 0;
}
#endif
