//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_LASER_H
#define ROVER_LASER_H

#include "Stoppable.h"
#include "Updatable.h"

class Laser : public Stoppable, public Updatable {
    protected:
        unsigned long timeTurnOn;
    public:
        void turnOn();
        void turnOff();
        bool isReady();
        unsigned long timeLeftForWarmUp();

        virtual void eStop() override;
        virtual ~Laser();
};


#endif //ROVER_LASER_H
