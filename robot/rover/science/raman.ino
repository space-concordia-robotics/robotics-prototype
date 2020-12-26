#include <Arduino.h>
#include <include/Laser.h>
#include <include/Fan.h>
#include <include/Funnel.h>
#include <include/Pump.h>
#include <include/commands/CommandCenter.h>
#include <include/commands/ScienceCommandCenter.h>
#include "include/Carousel.h"

const int NUMBER_OF_STOPPABLES = 5;
const int NUMBER_OF_UPDATABLES = 5;

Carousel* carousel = new Carousel();
Laser* laser = new Laser();
Fan* fan = new Fan();
Funnel* funnel = new Funnel();
Pump* pump = new Pump();

CommandCenter* commandCenter = new ScienceCommandCenter();

Stoppable* stoppables[NUMBER_OF_STOPPABLES] = {carousel, laser, fan, funnel, pump};
Updatable* updatables[NUMBER_OF_UPDATABLES] = {carousel, laser, fan, funnel, pump};
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

/**
 * Stops all systems immediately.
 */
void emergencyStop()
{
    for(auto & stoppable : stoppables)
    {
        stoppable->eStop();
    }
}