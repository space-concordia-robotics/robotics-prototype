//
// Created by cedric on 2020-10-10.
//

#ifndef ROVER_CAROUSEL_H
#define ROVER_CAROUSEL_H

#include "Stoppable.h"
#include "Updatable.h"

class Carousel : public Stoppable, public Updatable {
    protected:
        int currentCuvette;

    public:
        void home();
        void goToCuvette(int cuvetteId);
        void nextCuvette();
        void previousCuvette();
        int getCurrentCuvette() const { return currentCuvette; }

        virtual void eStop() override;
        virtual void update(unsigned long deltaMicroSeconds) override;
        virtual ~Carousel() {}
};


#endif //ROVER_CAROUSEL_H
