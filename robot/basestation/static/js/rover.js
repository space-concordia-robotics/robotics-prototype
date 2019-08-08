// for command thoughput limiting
const GAME_LOOP_PERIOD = 50
const CONTINUOUS_SERVO_PERIOD = 100
const POSITION_SERVO_PERIOD = 60
const SERVO_STOP = 93 // tested with front servos
const MIN_CONTINUOUS_SERVO_OFFSET = 4 // tested with front servos
const MAX_CONTINUOUS_SERVO_OFFSET = 20//30

const DRIVE_THROTTLE_TIME = 100
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
// constants for speed setting limits (absolute max: 45)
const MAX_THROTTLE_SPEED = 45
const MAX_STEERING_SPEED = 45
var lastCmdSent = 0
var lastFrontPosServoCmd = 0
var lastFrontContServoCmd = 0

var maxSoftThrottle = 25
var maxSoftSteering = 39
var maxFrontTiltPwm = 130
var minFrontTiltPwm = 50
var maxRearTiltPwm = 130
var minRearTiltPwm = 50

var throttle = 0 // how fast are the wheels turning in general
var steering = 0 // values further from 0 mean sharper turning radius
var spinning = 0 // for rotating around its centre
var frontTiltPwm = 90
var frontPanPwm = SERVO_STOP

var throttleIncrement = 1
var steeringIncrement = 1
var positionServoIncrement = 1
var continuousServoIncrement = 2
var continuousServoOffset = 20

var maxThrottleIncrement = 1
var maxSteeringIncrement = 1
var movementCommanded = false

sentZero = true // used to prevent the gui from sending wheel commands
sentFrontServoStop = true // used to prevent the gui from sending servo commands
sentRearServoStop = true // used to prevent the gui from sending servo commands

function printCommandsList () {
  appendToConsole("'ctrl-alt-p': ping odroid")
  appendToConsole("'p': ping rover mcu")
  appendToConsole("'q': emergency stop all motors")
  appendToConsole("'l': view key commands")
}

$(document).ready(function () {
  // camera servos

  // init camera servos
  let frontContServo = '!' + SERVO_STOP.toString()
  let rearContServo = '#' + SERVO_STOP.toString()
  sendRoverCommand(frontContServo)
  sendRoverCommand(rearContServo)

  // servo name: "Front camera positional tilt base"
  $('#camera-front-lpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand(
        // TODO: add more validation for input box
        '!' + $('#servo-val').val()
      )
    }
  })

  $('#camera-front-rpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('!' + $('#servo-val').val())
    }
  })

  // servo name: "Front camera Side continuous servo"
  $('#camera-front-tilt-up-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('@' + $('#servo-val').val())
    }
  })

  $('#camera-front-tilt-down-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('@' + $('#servo-val').val())
    }
  })

  // servo name: "Rear camera positional tilt base"
  $('#camera-back-lpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('#' + $('#servo-val').val())
    }
  })

  $('#camera-back-rpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('#' + $('#servo-val').val())
    }
  })

  // servo name: "Rear camera Side continuous servo"
  $('#camera-back-tilt-up-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('$' + $('#servo-val').val())
    }
  })

  $('#camera-back-tilt-down-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('$' + $('#servo-val').val())
    }
  })

  $('#reboot-button').on('click', function (event) {
    event.preventDefault()
    sendRoverCommand('reboot')
  })

  $('#list-all-cmds').on('click', function(event){
    event.preventDefault()
    printCommandsList()
  })

  $('#stop-all-motors').on('click', function(event){
    event.preventDefault()
    sendRoverCommand('stop')
  })

  $('#activate-rover-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#toggle-rover-listener-btn').is(':checked')) {
      appendToConsole('Rover listener not yet activated!')
    } else if ($('#activate-rover-btn').is(':checked')) {
      sendRequest("Rover", 'activate', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[0]) {
          $('#activate-rover-btn')[0].checked = true
        }
      })
    } else {
      // 'deactivated' needs to be handled differently since it takes 45 secconds
      sendRequest("Rover", 'deactivate', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[0]) {
          $('#activate-rover-btn')[0].checked = false
        }
      })
    }
  })
  $('#toggle-rover-listener-btn').on('click', function (event) {
    event.preventDefault()
    let serialType = $('#serial-type')
      .text()
      .trim()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-rover-listener-btn').is(':checked')) {
      if (
        $('button#mux')
          .text()
          .includes('Rover')
      ) {
        requestTask(
          'rover_listener',
          1,
          '#toggle-rover-listener-btn',
          function (msgs) {
            printErrToConsole(msgs)
            if (msgs[0]) {
              $('#toggle-rover-listener-btn')[0].checked = true
            } else {
              $('#toggle-rover-listener-btn')[0].checked = false
            }
          },
          serialType
        )
      } else {
        appendToConsole(
          'Cannot turn rover listener on if not in rover mux channel!'
        )
      }
    } else {
      // closing rover listener
      requestTask(
        'rover_listener',
        0,
        '#toggle-rover-listener-btn',
        function (msgs) {
          printErrToConsole(msgs)
          if (msgs.length == 2) {
            console.log('msgs[1]', msgs[1])
            if (msgs[1].includes('already running')) {
              $('#toggle-rover-listener-btn')[0].checked = true
            } else {
              $('#toggle-rover-listener-btn')[0].checked = false
            }
          } else {
            if (msgs[0]) {
              $('#toggle-rover-listener-btn')[0].checked = true
            } else {
              $('#toggle-rover-listener-btn')[0].checked = false
            }
          }
        },
        serialType
      )
    }
  })

  $('#send-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#antenna-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-start-dir-input').val()) {
      appendToConsole('bearing field empty!')
      goodInput = false
    }
    if (goodInput) {
      let initialLatitude = $('#antenna-latitude-input').val()
      let initialLongitude = $('#antenna-longitude-input').val()
      let initialBearing = $('#antenna-start-dir-input').val()
      antenna_latitude.set(parseFloat(initialLatitude))
      antenna_longitude.set(parseFloat(initialLongitude))
      antenna_start_dir.set(parseFloat(initialBearing))
      $('#antenna-latitude').text(initialLatitude)
      $('#antenna-longitude').text(initialLongitude)
      $('#antenna-start-dir').text(initialBearing)
      $('#antenna-inputs').hide()
      $('#antenna-unchanging').show()
    }
  })
  $('#change-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    $('#antenna-inputs').show()
    $('#antenna-unchanging').hide()
  })

  $('#send-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#goal-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#goal-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (goodInput) {
      let desiredLatitude = $('#goal-latitude-input').val()
      let desiredLongitude = $('#goal-longitude-input').val()
      goal_latitude.set(parseFloat(desiredLatitude))
      goal_longitude.set(parseFloat(desiredLongitude))
      $('#goal-latitude').text(desiredLatitude)
      $('#goal-longitude').text(desiredLongitude)
      $('#goal-inputs').hide()
      $('#goal-unchanging').show()
    }
  })
  $('#change-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    $('#goal-inputs').show()
    $('#goal-unchanging').hide()
  })
})

// KEYBOARD EVENTS
// odroid ping
document.addEventListener('keydown', function (event) {
  if (
    event.ctrlKey &&
    event.altKey &&
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME &&
    !$('#servo-val').is(':focus')
  ) {
    pingDevice('Odroid')
    lastCmdSent = new Date().getTime()
  }
})
// rover mcu ping
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME &&
    !$('#servo-val').is(':focus')
  ) {
    pingDevice('Rover')
    lastCmdSent = new Date().getTime()
  }
})
// print commands list
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyL' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME &&
    !$('#servo-val').is(':focus')
  ) {
    $('button#list-all-cmds').css('background-color', 'rgb(255, 0, 0)')
    printCommandsList()
    lastCmdSent = new Date().getTime()
  }
})

// commands to change speed settings, get buffered serial messages
$(document).keydown(function (e) {
  if (!$('#servo-val').is(':focus')) {
    switch (e.which) {
      case 73: // 'i' --> increase max throttle
        lightUp('#max-throttle-increase > button')
        maxSoftThrottle += maxThrottleIncrement
        if (maxSoftThrottle > MAX_THROTTLE_SPEED) {
          maxSoftThrottle = MAX_THROTTLE_SPEED
        }
        $('#max-throttle-speed').text(maxSoftThrottle)
        lastCmdSent = new Date().getTime()
        break

      case 74: // 'j' --> decrease max steering
        lightUp('#max-steering-decrease > button')
        maxSoftSteering -= maxSteeringIncrement
        if (maxSoftSteering < 0) {
          maxSoftSteering = 0
        }
        $('#max-steering-speed').text(maxSoftSteering)
        lastCmdSent = new Date().getTime()
        break

      case 75: // 'k' --> increase max steering
        lightUp('#max-steering-increase > button')
        maxSoftSteering += maxSteeringIncrement
        if (maxSoftSteering > MAX_STEERING_SPEED) {
          maxSoftSteering = MAX_STEERING_SPEED
        }
        $('#max-steering-speed').text(maxSoftSteering)
        lastCmdSent = new Date().getTime()
        break

      case 76: // 'l' --> list all commands
        printCommandsList()
        lastCmdSent = new Date().getTime()
        break

      case 81: // 'q' --> stop
        sendRoverCommand('stop')
        break

      case 85: // 'u' --> decrease max throttle
        lightUp('#max-throttle-decrease > button')
        maxSoftThrottle -= maxThrottleIncrement
        if (maxSoftThrottle < 0) {
          maxSoftThrottle = 0
        }
        $('#max-throttle-speed').text(maxSoftThrottle)
        lastCmdSent = new Date().getTime()
        break

      default:
        return // exit this handler for other keys
    }
    e.preventDefault() // prevent the default action (scroll / move caret)
  }
})

// no throttling necessary as since keydown events are throttled
// those keys will not change color and the following code will only set it to it's default color
$(document).keyup(function (e) {
  switch (e.which) {
    case 79:
      dim('#stop-motors-btn')
      break
    case 65: // left
      dim('#rover-left > button')
      break
    case 87: // up
      dim('#rover-up > button')
      break
    case 68: // right
      dim('#rover-right > button')
      break
    case 83: // down
      dim('#rover-down > button')
      break

    case 73: // increase throttle
      dim('#max-throttle-increase > button')
      break
    case 85: // decrease throttle
      dim('#max-throttle-decrease > button')
      break
    case 75: // increase steering
      dim('#max-steering-increase > button')
      break
    case 74: // decrease steering
      dim('#max-steering-decrease > button')
      break

    case 76: // list all rover cmds
      dim('button#list-all-rover-cmds')
      break

    default:
      return // exit this handler for other keys
  }
  e.preventDefault() // prevent the default action (scroll / move caret)
})

// GAME LOOP CONTROL

var keyState = {}
window.addEventListener(
  'keydown',
  function (e) {
    keyState[e.keyCode || e.which] = true
  },
  true
)
window.addEventListener(
  'keyup',
  function (e) {
    keyState[e.keyCode || e.which] = false
  },
  true
)

function handlePositionServo (
  pwmVal, minPwm, maxPwm,
  upKey, downKey,
  upBtn, downBtn,
) {
    let newCommand = false
    if (keyState[upKey]) {
      lightUp(upBtn)
      if (pwmVal > minPwm) {
        pwmVal -= positionServoIncrement
      }
      newCommand = true
    }
    else if (keyState[downKey]) {
      lightUp(downBtn)
      if (pwmVal < maxPwm) {
        pwmVal += positionServoIncrement
      }
      newCommand = true
    }
    return [newCommand, pwmVal]
}

function handleContinuousServo (
  pwmVal,
  leftKey, rightKey,
  leftBtn, rightBtn
) {
  if (keyState[leftKey] && !$('#servo-val').is(':focus')) {
    lightUp(leftBtn)
    if (pwmVal < SERVO_STOP + continuousServoOffset) {
      pwmVal += continuousServoIncrement
    }
  }
  else if (keyState[rightKey] && !$('#servo-val').is(':focus')) {
    lightUp(rightBtn)
    if (pwmVal > SERVO_STOP - continuousServoOffset) {
      pwmVal -= continuousServoIncrement
    }
  }
  else { // decelerate
    if (pwmVal < SERVO_STOP) {
      pwmVal += continuousServoIncrement
    }
    else if (pwmVal > SERVO_STOP) {
      pwmVal -= continuousServoIncrement
    }
    else {
      ; // do nothing, you've stopped
    }
  }
  return pwmVal
}

function gameLoop () {
  /*
  gameloop thought: what if we want more fine control on how fast the
  steering or the throttle ramps up? What if we want the rate of one
  to be different from that of the other? To me the solution is to
  have the rate (time) at which the commands are actually sent to be
  decoupled from the rate at which the steering or throttle changes
  */
  if (millisSince(lastCmdSent) > GAME_LOOP_PERIOD) {
    /* CAMERA SERVO CONTROL */
    //TODO: check if the position servo code is the same for rear servo or if
    // the directions must be reversed, because that may be annoying
    //TODO: same thing but for the continuous servo code
    //WARNING: the position servo limits are probably off
    //WARNING: currently I assume that 90 deg is the home position but this may not be the case
    //WARNING: the current implementation allows commands to two different servos to occur back-to-back
    //with no delay. this may be bad, consider something that only allows one command per x ms

    // front camera position servo (up/down, servo 1)
    if ( (millisSince(lastFrontPosServoCmd) > POSITION_SERVO_PERIOD) && !$('#servo-val').is(':focus') ) {
      let returnVals = handlePositionServo (
        frontTiltPwm, minFrontTiltPwm, maxFrontTiltPwm,
        111, 104, // numpad '/' and '8'
        '#camera-front-tilt-up-btn', '#camera-front-tilt-down-btn'
      )
      if (returnVals[0]) {
        frontTiltPwm = returnVals[1]
        $('#front-tilt-pwm').text(frontTiltPwm)
        lastFrontPosServoCmd = new Date().getTime()
        sendRoverCommand('@' + frontTiltPwm.toString())
      }
    }
    // Front camera continuous servo (left/right, servo 2)
    if ( (millisSince(lastFrontContServoCmd) > CONTINUOUS_SERVO_PERIOD) && !$('#servo-val').is(':focus') ){//> CONTINUOUS_SERVO_PERIOD){
      frontPanPwm = handleContinuousServo (
        frontPanPwm,
        103, 105, // numpad '7' and '9'
        '#camera-front-lpan-btn', '#camera-front-rpan-btn'
      )
      // check whether or not to send a new command for the continuous servo
      if (frontPanPwm == SERVO_STOP && sentFrontServoStop) {
        ;
      }
      else {
        let frontPan = SERVO_STOP
        if (frontPanPwm > SERVO_STOP && frontPanPwm < SERVO_STOP + MIN_CONTINUOUS_SERVO_OFFSET) {
          frontPan = SERVO_STOP + MIN_CONTINUOUS_SERVO_OFFSET
        }
        else if (frontPanPwm < SERVO_STOP && frontPanPwm > SERVO_STOP - MIN_CONTINUOUS_SERVO_OFFSET) {
          frontPan = SERVO_STOP - MIN_CONTINUOUS_SERVO_OFFSET
        }
        else {
          frontPan = frontPanPwm
        }
        $('#front-pan-pwm').text(frontPan)
        lastFrontContServoCmd = new Date().getTime()
        sendRoverCommand('!' + frontPan.toString())
        if (frontPanPwm != SERVO_STOP) {
          sentFrontServoStop = false
        }
        else {
          sentFrontServoStop = true
        }
      }
    }

    /*ROVER WHEEL CONTROL*/
    // 'd' --> rover right
    if (keyState[68] && !$('#servo-val').is(':focus')) {
      lightUp('#rover-right > button')
      if (steering < 0) {
        steering += 3 * steeringIncrement
      } else {
        steering += steeringIncrement
      }
      if (steering > maxSoftSteering) {
        steering = maxSoftSteering
      }
      // lastCmdSent = new Date().getTime()
    }
    // 'a' --> rover turn left
    else if (keyState[65] && !$('#servo-val').is(':focus')) {
      lightUp('#rover-left > button')
      if (steering > 0) {
        steering -= 3 * steeringIncrement
      } else {
        steering -= steeringIncrement
      }
      if (steering < -maxSoftSteering) {
        steering = -maxSoftSteering
      }
      // lastCmdSent = new Date().getTime()
    }
    // return to no steering angle
    else {
      if (steering < 0) {
        steering += steeringIncrement
      } else if (steering > 0) {
        steering -= steeringIncrement
      }
    }
    // 'w' --> rover forward
    if (keyState[87] && !$('#servo-val').is(':focus')) {
      lightUp('#rover-up > button')
      if (throttle < 0) {
        throttle += 3 * throttleIncrement
      } else {
        throttle += throttleIncrement
      }
      if (throttle > maxSoftThrottle) {
        throttle = maxSoftThrottle
      }
      // lastCmdSent = new Date().getTime()
    }
    // 's' --> rover back
    else if (keyState[83] && !$('#servo-val').is(':focus')) {
      lightUp('#rover-down > button')
      if (throttle > 0) {
        throttle -= 3 * throttleIncrement
      } else {
        throttle -= throttleIncrement
      }
      if (throttle < -maxSoftThrottle) {
        throttle = -maxSoftThrottle
      }
      // lastCmdSent = new Date().getTime()
    }
    // decelerate
    else {
      if (throttle < 0) {
        throttle += throttleIncrement
      } else if (throttle > 0) {
        throttle -= throttleIncrement
      } else {
        // do nothing, you're at 0
      }
    }
    if (throttle == 0 && sentZero) {
      // the rover is stopped
      if (steering == 0) {
      } // do nothing
      else {
        // turn rover in place
        /*
        The trick here is that we want `throttle` to be 0 to not
        interfere with previously written code. My idea was to therefore
        have a variable called `spinning` which does the same thing
        but for turning on itself when the w/s keys aren't being pressed.

        BUT, in a perfect world, `spinning` will accelerate the exact
        same way that throttle does.

        PLUS, ideally we can easily switch in and out of turning on
        ourselves and moving while turning because otherwise as soon
        as the switch happens, we'll jump back to 0 throttle/spinning...
        Unless the two variables are somehow swapped when the GUI
        realizes there's a change in modes.

        Last comment is that if thottle is negative does that reverse
        the direction that the rover spins compared to if it was positive?
        */
        spinning = maxSoftThrottle // perhaps needs to be commented when above is solved
        if (keyState[68]) {
          // 'd' --> rover right
          steering = MAX_STEERING_SPEED
          // do stuff with `spinning`
          // lastCmdSent = new Date().getTime()
        }
        else if (keyState[65]) {
          // 'a' --> rover turn left
          steering = -MAX_STEERING_SPEED
          // do stuff with `spinning`
          // lastCmdSent = new Date().getTime()
        }
        else {
          steering = 0
          spinning = 0
        }
        $('#throttle-speed').text(spinning)
        let cmd = spinning.toString() + ':' + steering.toString()
        sendRoverCommand(cmd)
        lastCmdSent = new Date().getTime()
      }
    }
    else {
      $('#throttle-speed').text(throttle)
      // the following stops sending commands if it already sent 0 throttle
      let cmd = throttle.toString() + ':' + steering.toString()
      sendRoverCommand(cmd)
      lastCmdSent = new Date().getTime()
      if (throttle != 0) {
        sentZero = false
      }
      else {
        sentZero = true
      }
    }
    $('#steering-speed').text(steering)
  }
  setTimeout(gameLoop, 5)
}

gameLoop()
