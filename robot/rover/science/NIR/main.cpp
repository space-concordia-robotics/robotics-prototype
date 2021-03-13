#include <iostream>
#include "Spectrometer.h"

using namespace std;
Spectrometer spec;
int main(int argc, const char **argv)
{
    if (!spec.init())
    {
        cout << "Cannot initialize USB device!" << endl;
        return -1;
    }
    //spec.scan();
    spec.deinit();
    return 0;
}
