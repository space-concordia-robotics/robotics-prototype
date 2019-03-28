/*
  TODO:

open loop speed and closed loop max speed are different????
-this is because closed loop has a max val whereas open loop has a fixed one, but these could maybe be the same???
stepper doesn't have speed the way servos and dcs do??????
-this is because stepper calculates speed using durations, but this means openloopgain and openloopspeed mean nothing for it???
  
  -suggestion 1 pinmode input, 2 is micros pwm instead of analogwrite (timer is bad?)
  -(always) implement error checking, remove parts later on if it's too slow (tatum doubts)
  -(always) clean up code, comment code
  -(always) make sure variables modded in ISRs and used externally are volatile
  -(done-ish) PID implemented for DC motor but need to adjust all variables
  -(depends on wiring) determine the actual clockwise and counter-clockwise directions of motors based on their wiring in the arm itself
  -(next) clean up parsing code to make it easier to read and reduce repetition

  -(done-ish) determine motor angles thru encoder interrupts
   -(done-ish) solve encoder direction change issue? not necessary
   -(issue) interrupt functions must be defined outside of classes...
   -(now) confirm all the pins will work with interrupts and not stepping on each other
   -(done-ish) encoder resolution and gear ratios determined for most motors but not m1,m5,m6
   -(done-ish) can determine motor angles for dc and steppers, not servos
   -(now) deal with overflow of encoderCount.. does it ever reach max value?
   -(later) determine whether it's worth it to use the built in quadrature decoders

  -(done-ish) simultaneous motor control with timers
   -(done-ish) software interrupts can take input from open loop control but need to refine everything and finish closed loop control
   -more comments in next comment block

  -(next step) external interrupts for limit switches
   -(done) rewrite all the register bit variables to use teensy registers for limit switch interrupts
   -(done) implement limit switch interrupts to move motor back into soft angle range
   -(now) implement homing function

  -(next next step) timers
   -systick is normally a heartbeat type thing
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
   -implement heartbeat after deciding on best timer for it
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -currently using whatever is connected to the pwm pins for timing pwm
    -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
    -ftm+tpm has quadrature decoder?
    -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm

  -even more notes in google drive
*/
/*
   todo: add comments everywhere!!!!
  also set the interrupt priority...

  perhaps pinsetup.h & pinsetup.cpp should be changed to motorsetup or just setup as it's also got angle limits and gear ratios
   technically i can use my motorarray shorthand to shorten the switch/case thing significantly
   gotta figure out the correspondence between direction pin's high/low and motor rotation direction

   I need to figure out where it's a good idea to disable interrupts so htat I don't read a value while it's being modified

  6) angleTolerance is motorPID attribute and not motor attribute?
  7) openloopspeed right now is set in setup... but this defeats the purpose of initializing the speed to 0 in the constructor... i set it to 0 as a precaution but technically the motors shouldn't turn anyway because movementDone controls that. so maybe I can just initialize to 50 and not have it in the setup?
  10) should setOutputLimits be restricted to just pidcontroller or should it be something for all motors...?
      but then i set openloopspeed to 50 in the setup()???? i need to rethink the velocity vs speed vs direction stuff!
  17) perhaps the initialization of motor angle parameters should be its own function
  18) perhaps the initialization of the motor's pins should also be its own function
  19) perhaps the constructor should take a series of structs, one for each type
  20) fix that issue with discrepancy that's related to the periodic angle checks
  23) stepper motor angle checks for open loop control remains to be fixed, updated, implemented

  DEAL WITH ANGLERESOLUTION VARIABLE IN PIDCONTROLLER

  24) maybe we should have some way of covering the limit switches - what if someone accidentally presses one?
  thushanth said the way that 3d printers work is they move away then slowly move back to hit the switch again as a check

  25) there should also be a check so that if the motor is moving away from the goal position or has been moving for a while without reaching the expected angle, it stops... like a timeout

  26) what do I do for angles over 360? do i keep counting up? do i keep count of how many rotations I've done?

  27) what happens if a new command tells the motor to turn in opposite direction? abrupt changes are bad. if the stepper is trying to turn but hasn't gotten anywhere there should be a check in the microcontroller that there's an issue (there can also be a check in the gui)
  the base station/odroid should be in charge of ramping up angles and the teensy should just go to them
  perhaps if a large angle is requested there should still be a way to stop it though

  for encoder interrupt testing on the teensy, send 3.3v to all interrupt pins simultaneously to test thing?
  floating point math doesn't seem bad, but at worst, convert float to int before motor control and do int math inside interrupts

  we don't want to miss messages, we need to know if messages were missed, for example know starting and ending message characters, we need some kind of confirmation anyway

  quadrature on ftm1,2: pins 3/4,29/30: cant use for pwm anymore
  quadrature on tpm1,2: pins 16/17, (tpm2 not implemented in teensy?)

  for steppers the check is more to do with whether the amount of steps has been reached and if not, try to step more... whereas for dc they control voltage so the issue there is whether the pid tries to go to max voltage

  fuzzy logic followed by pid? just fuzzy logic? different pid constants based on how large the error is? small-medium error has large constants and medium high has smaller ones so it doesn't overreact?

  according to my googling, many people say that for steppers it's better to control them open-loop style until the very end of the motion, at which point you correct for errors based on the angle difference. This is partly because of the step angle resolution compared to the encoder resolution and pid output resolution causing instability since the PID tries to overcompensate too quickly?

  But since we have a gear/belt reduction, and since we can programmatically decide which errors we can ignore, maybe it's not as big of a deal and PID is okay... after all, we'd have to manually implement stepper acceleration if we don't use a PID, although code examples do already exist.

  make sure all the math happens behind the scenes
  make sure the math calculations are written correctly and calculate as quickly as possible
  calcCurrentAngle for example

  for motor speed, use encoder to calculate speed like i did in MotorCharacterization
  speed should be calculated the way angle is, only when necessary in interrupt loop
  speed pid takes speed as measurement and compares to desired speed
  what's the relationship between position and speed that helps design the position pid

  different types of ramping profiles - trapezoid vs quintic polynomial
*/
