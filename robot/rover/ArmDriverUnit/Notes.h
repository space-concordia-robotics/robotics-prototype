/*
  24) maybe we should have some way of covering the limit switches - what if someone accidentally presses one?
  thushanth said the way that 3d printers work is they move away then slowly move back to hit the switch again as a check

// the following works if the switches are debounced by the hardware
  /*
    #if defined(LIM_SWITCH_FALL)
    #define LIM_SWITCH_DIR FALLING
    #elif defined(LIM_SWITCH_RISE)
    #define LIM_SWITCH_DIR RISING
    #endif
  */
/*
timers:
  -watchdog timer to be added into my code soonish to reset the teensy if it's hanging
   -heartbeat will be implemented using millis as millis uses systick so do we really need to do interrupts?
   -lptmr runs even on low power mode, maybe this should be heartbeat instead?
   -pit is used for intervaltimer objects, there are 4 and they work like interrupts
   -pwm: teensy has 6 16bit pwm timers and apparently 22 total pwm options:
    -currently using whatever is connected to the pwm pins for timing pwm
    -teensy pwm page mentions ftm and tpm timers which aren't mentioned in the page with other timesr
    -ftm+tpm has quadrature decoder?
    -tpm is 2-8 channel timer with pwm, 16bit counter, 2 channels for pwm

  we don't want to miss messages, we need to know if messages were missed, for example know starting and ending message characters, we need some kind of confirmation anyway
  for steppers the check is more to do with whether the amount of steps has been reached and if not, try to step more... whereas for dc they control voltage so the issue there is whether the pid tries to go to max voltage
  fuzzy logic followed by pid? just fuzzy logic? different pid constants based on how large the error is? small-medium error has large constants and medium high has smaller ones so it doesn't overreact?
  according to my googling, many people say that for steppers it's better to control them open-loop style until the very end of the motion, at which point you correct for errors based on the angle difference. This is partly because of the step angle resolution compared to the encoder resolution and pid output resolution causing instability since the PID tries to overcompensate too quickly?
  But since we have a gear/belt reduction, and since we can programmatically decide which errors we can ignore, maybe it's not as big of a deal and PID is okay... after all, we'd have to manually implement stepper acceleration if we don't use a PID, although code examples do already exist.
  use something based on fauzi's solution for motor speed
*/
