#include "Carousel.h"
Carousel carousel;
#include "AUnit.h"
test(correct) {
        carousel.eStop();
        int x = 1;
        assertEqual(x, 1);
}

test(incorrect) {
        int x = 1;
        assertNotEqual(x, 1);
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