#include <Arduino.h>
#include "include/Carousel.h"

const int NUMBER_OF_STOPPABLES = 5;
const int NUMBER_OF_UPDATABLES = 5;

Carousel* carousel;
Stoppable* stoppable[NUMBER_OF_STOPPABLES] = {carousel};
Updatable* updatables[NUMBER_OF_UPDATABLES] = {carousel};
float time = millis();

void loop()
{
    for(int i = 0; i < NUMBER_OF_UPDATABLES; i++)
    {
        float delta = millis() - time;
        updatables[i]->update(delta);
    }

    time = millis();
}

void setup()
{}