#!/usr/bin/python


import wiringpi2 as wpi
import time

i2c_dev= "/dev/i2c-1"
i2c_addr = 0x62

wpi.wiringPiSetup()
i2c_fd = wpi.wiringPiI2CSetupInterface(i2c_dev, i2c_addr)

writeResult = wpi.wiringPiI2CWriteReg8(i2c_fd, 0x1e, 0x00)
print("write result: " + str(writeResult));

reg_addr = 0x00

while(reg_addr <= 101):
    #print("testing address: " + str(reg_addr))
    if(reg_addr != 30):
        output = wpi.wiringPiI2CReadReg8(i2c_fd, (reg_addr))
        print(hex(reg_addr) + ":" + "{0:b}".format(output))
        #print("\n")
        time.sleep(0.01)
    reg_addr += 1

#print("before: " + str(i2c_fd))

#wpi.wiringPiI2CRead(i2c_fd)

#wpi.wiringPiI2CWriteReg8(i2c_fd, 0x00, 0x00)

#output = wpi.wiringPiI2CReadReg8(i2c_fd, 0x01)

#wpi.wiringPiI2CWriteReg16(i2c_fd, 0x10, 0x1234)

#print("after: " + str(i2c_fd))
#print("{0:b}".format(output)) 





