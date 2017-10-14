## lidar

### Current Tasks

- Run a test of current LIDAR camera
- Test using previous teams code, generate a report on it

## How to connect lidar module to odroid (no servos involved)

- Connect power wire (red wire sticking out of capacitor) to pin 2
- Connect ground wire (black wire sticking out of capacitor) to pin 6
- Connect I2C wires (Lidar I2C SCA from [lidar-board-interface-thingy.png](https://github.com/space-concordia-robotics/robotics-prototype/blob/34efe5dc4cd1a26d1a3c19e78ccac35cd4398f59/lidar/images/lidar-board-interface-thingy.png)) to pins 3 and 5. Not sure if the order matters there.