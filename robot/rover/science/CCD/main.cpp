#include <iostream>
#include "ni-visa/visa.h"

#define MAX_CNT 200

int main() {

    ViStatus status;
    ViSession defaultRM, instr;
    ViUInt32 retCount;
    ViChar	buffer[MAX_CNT];
    ViPBuf buf[MAX_CNT];
    /* Buffer for string I/O */
    std::cout << "Hello, World!" << std::endl;

    status = viOpenDefaultRM(&defaultRM);
    if (status < VI_SUCCESS) {
      return -1;
    }

    status = viOpen(defaultRM, "GPIB0::1::INSTR", VI_NULL, VI_NULL, &instr);
    std::cout << status;
    status = viSetAttribute(instr, VI_ATTR_TMO_VALUE, 5000);
    status = viWrite(instr, reinterpret_cast<ViConstBuf>("*IDN?\n"), 6, &retCount);
    //status = viRead(instr, reinterpret_cast<ViPBuf>(buf), (ViUInt32)MAX_CNT, &retCount);
    status = viRead(instr, reinterpret_cast<ViPBuf>(buffer), MAX_CNT, &retCount);
    status = viClose(instr);
    status = viClose(defaultRM);
    return 0;
}

