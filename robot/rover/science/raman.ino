#include <Arduino.h>
#include <include/Laser.h>
#include <include/Fan.h>
#include <include/Funnel.h>
#include <include/Pump.h>
#include "CommandCenter.h"
#include "include/commands/ScienceCommandCenter.h"
#include "include/Carousel.h"
#include <cstdint>

const uint8_t NUMBER_OF_STOPPABLES = 5;
const uint8_t NUMBER_OF_UPDATABLES = 5;

const uint8_t TX_TEENSY_4_0_PIN = 1;
const uint8_t RX_TEENSY_4_0_PIN = 0;
const uint8_t ENABLE_PIN = 10; // THIS IS A PLACE HOLDER UNTIL FLOW CONTROL CAN BE IMPLEMENTED

void updateSystems();

Carousel* carousel = new Carousel();
Laser* laser = new Laser();
Fan* fan = new Fan();
Funnel* funnel = new Funnel();
Pump* pump = new Pump();

internal_comms::CommandCenter* commandCenter = new ScienceCommandCenter();

Stoppable* stoppables[NUMBER_OF_STOPPABLES] = {carousel, laser, fan, funnel, pump};
Updatable* updatables[NUMBER_OF_UPDATABLES] = {carousel, laser, fan, funnel, pump};
unsigned long time = micros();

void loop()
{
    if(Serial.available() > 0)
        commandCenter->readCommand();

    updateSystems();
}

void setup()
{
    commandCenter->startSerial(TX_TEENSY_4_0_PIN, RX_TEENSY_4_0_PIN, ENABLE_PIN);
}

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
