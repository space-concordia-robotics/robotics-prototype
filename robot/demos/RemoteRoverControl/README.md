# Remote Rover Control Demo

## Setup
In parentheses will be what I used for testing specifically

- Base station 
  - Any computer running Linux (space lab computer running ubuntu 16.04, laptop from home running ubuntu 18.04)
- Rover: 
  - Any computer running Linux (Odroid XU4Q running ubuntuMATE)
  - Arduino (Arduino Mega 2560)
  - Motor Driver (Sabertooth 2x25)
  - DC Motor (AndyMark Gearmotor am-2971)
  - Power supply (ABRA DC Power Supply AB-3000 set to 12V, 1A)
  - Low pass filter (RC circuit with R = 10kOhm, C = 0.1 microFarad)

Make sure to upload the `SerialMoveDCMotor.ino` to your arduino board, and if you're using a different board make sure to adjust it accordingly (such as if you need to use different pins).
Then plug your arduino into your rover computer via USB.

[Here](https://drive.google.com/open?id=1hE31jaaIMZ-enYLBaENh1Ro08JTc9NsJTF_GJ_DKvyk) is a guide to setup a DC motor with arduino and sabertooth motor driver.

Both computers need to be connected to the same network.

## Running the demo
Always run the scripts in this order:
1. On the rover computer run `./ServerListener.py`
2. On the base-station computer run `./ClientSender.py <ip base station computer>`

After step 2, wait until you see "Ready for sending drive commands!"
Now you may use 'w' for moving the motor forward, 's' for back, 'q' to terminate the connection.

## Known Issues
- If your basestation and rover computers are both connected to `ConcordiaUniversity`, then you will not be able to run this
demo properly if your basestation is connected to the network via WiFi. It *will* however work if the basestation is connected using ethernet.
A possible work around for this is either using a Wifi hotspot to connect both devices to or to create WLAN for both devices to connect.
~~These possible work arounds have not yet been tested.~~ Using a Wifi hotspot was tested and works sending commands from a laptop when both devices are
connected to the same hotspot. Though during a public demo in EV, due to the high density of people there was a lot of interference (compared to the space lab)
and the response times of the connection were to slow to properly demo.
- After "spamming" the controls to test how much they can handle, it seemed that after a certain amount of time the UDP packets being delivered
slowed down significantly. This is not 100% confirmed, but may be an issue related to ConcordiaUniversity network configuration. Perhaps if a certain threshold is reached,
these types of signal streams are throttled by the network.
