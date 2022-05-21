//
// Created by cedric on 2020-10-14.
//

#ifndef ROVER_FUNNEL_H
#define ROVER_FUNNEL_H


#include "Updatable.h"
#include <cstdint>

class Funnel : public Updatable {
    protected:
        unsigned long funnelTimeStarted;
        unsigned long timeToFunnel;
        bool isMoving;
    public:
        Funnel();
        void startFunnel();
        void runFor(float ms);
        void stop();
        bool moving();
        virtual void update(unsigned long) override;
};


#endif //ROVER_FUNNEL_H
