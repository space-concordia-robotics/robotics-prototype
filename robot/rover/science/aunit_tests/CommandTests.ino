#line 2 "Print64Test.ino"
#include "include/commands/ScienceCommandCenter.h"
#include <AUnit.h>
#include <aunit/fake/FakePrint.h>
ScienceCommandCenter commandCenter;

using namespace aunit;
using namespace aunit::fake;
using namespace aunit::internal;

test(Print64, fakePrint) {
    FakePrint fakePrint;

    fakePrint.print('a');
    assertEqual("a", fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print("01234567890123456789");
    assertEqual("01234567890123456789", fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print(123);
    assertEqual("123", fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print(10, 16);
    assertEqual("A", fakePrint.getBuffer());
    fakePrint.flush();

    // Don't know why print() for base==0 just prints out the lower 8-bits,
    // but that's what we need to assert.
    fakePrint.print(256 + 'A', 0);
    assertEqual("A", fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print(-1, 16);
    const char* expected = (sizeof(long) == 8) ? "FFFFFFFFFFFFFFFF" : "FFFFFFFF";
    assertEqual(expected, fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print(INT32_MIN);
    assertEqual("-2147483648", fakePrint.getBuffer());
    fakePrint.flush();

    fakePrint.print(-20.5);
    assertEqual("-20.50", fakePrint.getBuffer());
    fakePrint.flush();
}

void setup() {
#ifdef ARDUINO
    delay(1000); // Wait for stability on some boards, otherwise garage on Serial
#endif
    SERIAL_PORT_MONITOR.begin(115200);
    while (! SERIAL_PORT_MONITOR); // Wait until Serial is ready - Leonardo
}


void loop() {
    TestRunner::run();
}