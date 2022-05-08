//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_FUNNEL_H
#define ROVER_FUNNEL_H


#include "Updatable.h"
#include <cstdint>

class Funnel : public Updatable {
    protected:
        uint8_t funnelPWM;
    public:
        virtual void funnel();
        virtual void update(unsigned long) override;
        virtual ~Funnel();
};


#endif //ROVER_FUNNEL_H
