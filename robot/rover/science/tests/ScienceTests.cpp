#include "Carousel.h"
Carousel carousel;
#include "AUnit.h"
test(correct) {
        int x = 1;
        assertEqual(x, 1);
}

test(incorrect) {
        int x = 3;
        assertEqual(x, 3);
}

//----------------------------------------------------------------------------
// setup() and loop()
//----------------------------------------------------------------------------

void setup() {
    delay(1000); // wait for stability on some boards to prevent garbage Serial
    Serial.begin(115200); // ESP8266 default of 74880 not supported on Linux
}

void loop() {
    aunit::TestRunner::run();
}