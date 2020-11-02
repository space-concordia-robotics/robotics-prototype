//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_FAN_H
#define ROVER_FAN_H

#include "Updatable.h"
#include "Stoppable.h"

class Fan :  public Stoppable, public Updatable {

    protected:
        uint8_t fanPWM;
    public:
        void turnOn();
        void turnOff();

        virtual void update(unsigned long) override;
        virtual void eStop() override;
        virtual ~Fan();
};


#endif //ROVER_FAN_H
