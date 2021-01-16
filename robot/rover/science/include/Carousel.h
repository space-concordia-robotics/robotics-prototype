//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include "Stoppable.h"
#include "Updatable.h"
#include <cstdint>

class Carousel : public Stoppable, public Updatable {
    protected:
        uint8_t currentCuvette;

    public:
        void home();
        void goToCuvette(uint8_t cuvetteId);
        void nextCuvette();
        void previousCuvette();
        uint8_t getCurrentCuvette() const { return currentCuvette; }

        virtual void eStop() override;
        virtual void update(unsigned long deltaMicroSeconds) override;
        virtual ~Carousel();
};


#endif //ROVER_CAROUSEL_H
