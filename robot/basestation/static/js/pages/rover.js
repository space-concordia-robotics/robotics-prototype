
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
