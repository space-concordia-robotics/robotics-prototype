// for command thoughput limiting
const GAME_LOOP_PERIOD = 50
const DRIVE_THROTTLE_TIME = 100
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
// constants for speed setting limits (absolute max: 45)
const MAX_THROTTLE_SPEED = 45
const MAX_STEERING_SPEED = 45


let lastCmdSent = 0
let lastFrontPosServoCmd = 0
let lastFrontContServoCmd = 0
let lastRearPosServoCmd = 0
let lastRearContServoCmd = 0

let maxSoftThrottle = 25
let maxSoftSteering = 39

let throttle = 0 // how fast are the wheels turning in general
let steering = 0 // values further from 0 mean sharper turning radius
let spinning = 0 // for rotating around its centre

let throttleIncrement = 1
let steeringIncrement = 1

let maxThrottleIncrement = 1
let maxSteeringIncrement = 1
let movementCommanded = false

sentZero = true // used to prevent the gui from sending wheel commands

function printCommandsList () {
  appendToConsole("'ctrl-alt-p': ping odroid")
  appendToConsole("'p': ping rover mcu")
  appendToConsole("'q': emergency stop all motors")
  appendToConsole("'l': view key commands")
}

$(document).ready(function () {

  $('#reboot-button').on('click', function (event) {
    event.preventDefault()
    sendRoverCommand('reboot')
  })

  $('#list-all-cmds').on('click', function (event) {
    event.preventDefault()
    printCommandsList()
  })

  $('#stop-all-motors').on('click', function (event) {
    event.preventDefault()
    sendRoverCommand('stop')
  })

  $('#activate-rover-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#toggle-rover-listener-btn').is(':checked')) {
      appendToConsole('Rover listener not yet activated!')
    } else {
      if ($('#activate-rover-btn').is(':checked')) {
        sendRequest('Rover', 'activate', function (msgs) {
          printErrToConsole(msgs)
          if (msgs[0]) {
            $('#activate-rover-btn')[0].checked = true
          }
        })
      } else {
        // 'deactivated' needs to be handled differently since it takes 45 secconds
        sendRequest('Rover', 'deactivate', function (msgs) {
          printErrToConsole(msgs)
          if (msgs[0]) {
            $('#activate-rover-btn')[0].checked = false
          }
        })
      }
    }
  })

  $('#toggle-rover-pid-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#toggle-rover-listener-btn').is(':checked')) {
      appendToConsole('Rover listener not yet activated!')
    } else if ($('#toggle-rover-pid-btn').is(':checked')) {
      sendRequest('Rover', 'close-loop', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[1].includes('loop status is: CLose')) {
          $('#toggle-rover-pid-btn')[0].checked = true
          appendToConsole('Loop status: closed')
        }
      })
    } else {
      sendRequest('Rover', 'open-loop', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[1].includes('loop status is: Open')) {
          $('#toggle-rover-pid-btn')[0].checked = false
          appendToConsole('Loop status: open')
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
      // validate UART mode options are correct, let pass if USB mode selected
      if (
        ($('button#mux')
          .text()
          .includes('Rover') &&
          serialType == 'uart') ||
        serialType == 'usb'
      ) {
        requestTask(
          ROVER_LISTENER_TASK,
          STATUS_START,
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
          'UART MODE: Cannot turn rover listener on if not in rover mux channel!'
        )
      }
    } else {
      // closing rover listener
      requestTask(
        ROVER_LISTENER_TASK,
        STATUS_STOP,
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
})

// rover mcu ping
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME &&
    !$('#servo-val').is(':focus') && (!$('#serial-command-input').is(':focus'))
  ) {
    pingDevice('Rover')
    lastCmdSent = new Date().getTime()
  }
})

// Button coloring
const YELLOW = 'rgb(255, 249, 148)'
const GREEN = 'rgb(61, 127, 127)'

// commands to change speed settings, get buffered serial messages
$(document).keydown(function (e) {
  if ((!$('#servo-val').is(':focus')) && (!$('#serial-command-input').is(':focus'))) {
    switch (e.which) {
      case 73: // 'i' --> increase max throttle
        lightUp('#max-throttle-increase > button')
        $('#max-throttle-increase > button').css(
          'background-color',
          YELLOW)
        maxSoftThrottle += maxThrottleIncrement
        if (maxSoftThrottle > MAX_THROTTLE_SPEED) {
          maxSoftThrottle = MAX_THROTTLE_SPEED
        }
        $('#max-throttle-speed').text(maxSoftThrottle)
        lastCmdSent = new Date().getTime()
        break

      case 74: // 'j' --> decrease max steering
        lightUp('#max-steering-decrease > button')
        $('#max-steering-decrease > button').css(
          'background-color',
          YELLOW)
        maxSoftSteering -= maxSteeringIncrement
        if (maxSoftSteering < 0) {
          maxSoftSteering = 0
        }
        $('#max-steering-speed').text(maxSoftSteering)
        lastCmdSent = new Date().getTime()
        break

      case 75: // 'k' --> increase max steering
        lightUp('#max-steering-increase > button')
        $('#max-steering-increase > button').css(
          'background-color',
          YELLOW)
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
        $('#max-throttle-decrease > button').css(
          'background-color',
          YELLOW)
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
  if (!$('#serial-command-input').is(':focus')) {
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
  }
})

// GAME LOOP CONTROL

var keyState = {}
window.addEventListener(
  'keydown',
  function (e) {
    if (!$('#serial-command-input').is(':focus')) {
      keyState[e.keyCode || e.which] = true
    }
  },
  true
)
window.addEventListener(
  'keyup',
  function (e) {
    if (!$('#serial-command-input').is(':focus')) {
      keyState[e.keyCode || e.which] = false
    }
  },
  true
)

function gameLoop () {
  /*
  gameloop thought: what if we want more fine control on how fast the
  steering or the throttle ramps up? What if we want the rate of one
  to be different from that of the other? To me the solution is to
  have the rate (time) at which the commands are actually sent to be
  decoupled from the rate at which the steering or throttle changes
  */
  if (millisSince(lastCmdSent) > GAME_LOOP_PERIOD) {
    /* ROVER WHEEL CONTROL */
    // 'd' --> rover right
    if (keyState[68] && !$('#servo-val').is(':focus')) {
      lightUp('#rover-right > button')
      $('#rover-right > button').css(
        'background-color',
        YELLOW)
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
      $('#rover-left > button').css(
        'background-color',
        YELLOW)
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
      $('#rover-up > button').css(
        'background-color',
        YELLOW)
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
      $('#rover-down > button').css(
        'background-color',
        YELLOW)
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
        } else if (keyState[65]) {
          // 'a' --> rover turn left
          steering = -MAX_STEERING_SPEED
          // do stuff with `spinning`
          // lastCmdSent = new Date().getTime()
        } else {
          steering = 0
          spinning = 0
        }
        $('#throttle-speed').text(spinning)
        let cmd = spinning.toString() + ':' + steering.toString()
        sendRoverCommand(cmd)
        lastCmdSent = new Date().getTime()
      }
    } else {
      $('#throttle-speed').text(throttle)
      // the following stops sending commands if it already sent 0 throttle
      let cmd = throttle.toString() + ':' + steering.toString()
      sendRoverCommand(cmd)
      lastCmdSent = new Date().getTime()
      if (throttle != 0) {
        sentZero = false
      } else {
        sentZero = true
      }
    }
    $('#steering-speed').text(steering)
  }
  setTimeout(gameLoop, 5)
}
gameLoop()

// In any case
// the following code just makes the buttons stop lighting up
// when the user stops pressing the respective key

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyI') {
    $('#max-throttle-increase > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyJ') {
    $('#max-steering-decrease > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyK') {
    $('#max-steering-increase > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyU') {
    $('#max-throttle-decrease > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyD') {
    $('#rover-right > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyA') {
    $('#rover-left > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyW') {
    $('#rover-up > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyS') {
    $('#rover-down > button').css('background-color', GREEN)
  }
})
