//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_FUNNEL_H
#define ROVER_FUNNEL_H


#include "Stoppable.h"
#include "Updatable.h"

class Funnel : public Stoppable, public Updatable {
    protected:
        uint8_t funnelPWM;
    public:
        virtual void funnel();
        virtual void update(unsigned long) override;
        virtual void eStop() override;
        virtual ~Funnel();
};


#endif //ROVER_FUNNEL_H
