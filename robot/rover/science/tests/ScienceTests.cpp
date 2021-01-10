#line 2 "basic.ino"
#include "AUnit.h"
test(correct) {
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
    while(!Serial); // for the Arduino Leonardo/Micro only

    Serial.println(F("This test should produce the following:"));
    Serial.println(
            F("1 passed, 1 failed, 0 skipped, 0 timed out, out of 2 test(s).")
    );
    Serial.println(F("----"));
}

void loop() {
    aunit::TestRunner::run();
}