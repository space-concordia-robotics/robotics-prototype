#include <Arduino.h>
#include <include/Laser.h>
#include "include/Carousel.h"

const int NUMBER_OF_STOPPABLES = 5;
const int NUMBER_OF_UPDATABLES = 5;

Carousel* carousel;
Laser* laser;
Stoppable* stoppable[NUMBER_OF_STOPPABLES] = {carousel, laser};
Updatable* updatables[NUMBER_OF_UPDATABLES] = {carousel};
unsigned long time = micros();

void updateSystems();

void loop()
{
    updateSystems();
}

void setup()
{}

/**
 * Calculates delta for each system and calls their update method.
 * The delta calculation is inaccurate due to the fact that the delta
 * is not exact for each system. An improvement would be to have a separate dt
 * for each system.
 */
void updateSystems()
{
    for(auto & updatable : updatables)
    {
        unsigned long delta = micros() - time;
        updatable->update(delta);
    }

    time = micros();
}