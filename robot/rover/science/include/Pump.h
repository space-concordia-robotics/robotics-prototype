//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_PUMP_H
#define ROVER_PUMP_H


#include "Updatable.h"
#include "Stoppable.h"

class Pump : public Stoppable, public Updatable{
    protected:
        unsigned long pumpTimeElapsed;
        int pumpPWM;
    public:
        virtual void pump();
        virtual void backpump();

        virtual void eStop() override;

        virtual void update(unsigned long deltaMicroSeconds) override;

        virtual ~Pump();
};


#endif //ROVER_PUMP_H
