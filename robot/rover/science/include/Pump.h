//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_PUMP_H
#define ROVER_PUMP_H


#include "Updatable.h"
#include <cstdint>

class Pump : public Updatable{
    protected:
        unsigned long pumpTimeStarted;
        unsigned long timeToPump;
        uint8_t pumpPWM;
        bool isMoving;
    public:
        void pump();
        void backpump();
        void stop();
        void pumpFor(float ms);
        void backpumpFor(float ms);
        bool moving();

        virtual void update(unsigned long deltaMicroSeconds) override;
};


#endif //ROVER_PUMP_H
