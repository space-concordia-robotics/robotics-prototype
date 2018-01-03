## lidar

### Current Tasks (phase 3)

Sprint 1:
~~- Get basic measurements out of LIDAR (using Arduino)~~
- Assess accuracy of LIDAR readings compared to expected accuracy 

Sprint 2:
- Test processing measurement data using last years code/approach, integrate functioning version into new code base

## How to connect lidar module to odroid (no servos involved)

- Connect power wire (red wire sticking out of capacitor) to pin 2
- Connect ground wire (black wire sticking out of capacitor) to pin 6
- Connect I2C wires (Lidar I2C SCA from [lidar-board-interface-thingy.png](https://github.com/space-concordia-robotics/robotics-prototype/blob/34efe5dc4cd1a26d1a3c19e78ccac35cd4398f59/lidar/images/lidar-board-interface-thingy.png)). Grey wire (SDA) to pin 3 and green wire (SCL) to pin 5.



