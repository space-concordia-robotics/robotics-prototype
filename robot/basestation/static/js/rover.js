// @TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

// constants for speed setting limits (absolute max: 45)
const MAX_THROTTLE_SPEED = 45
const MAX_STEERING_SPEED = 39

// for command thoughput limiting
const DRIVE_THROTTLE_TIME = 25
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
const LISTENER_TOGGLE_THROTTLE = 3000
var lastCmdSent = 0
var maxSoftThrottle = 25
var maxSoftSteering = 39
var throttle = 0
var steering = 0
var throttleIncrement = 1
var steeringIncrement = 1
var maxThrottleIncrement = 1
var maxSteeringIncrement = 1
var movementCommanded = false


// for updating the toggle buttons if user pressed keyboard events triggered enabling/disabling
function enableRoverMotorsBtn () {
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).removeClass(
    'btn-danger off'
  )
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).addClass(
    'btn-success'
  )
  $('#enable-rover-motors-btn')[0].checked = true
}

function disableRoverMotorsBtn () {
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).addClass(
    'btn-danger off'
  )
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).removeClass(
    'btn-success'
  )
  $('#enable-rover-motors-btn')[0].checked = false
}

$(document).ready(function () {
  $('#ping-rover-mcu').on('click', function (event) {
    event.preventDefault()
    if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
      sendRoverRequest('ping', function (msgs) {})
      lastCmdSent = new Date().getTime()
    }
  })

  $('#toggle-rover-listener-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-rover-listener-btn').is(':checked')) {
      if (
        $('button#mux')
          .text()
          .includes('Rover')
      ) {
        requestTask('rover_listener', 1, '#toggle-rover-listener-btn', function (
          msgs
        ) {
          console.log('msgs[0]', msgs[0])
          if (msgs.length == 2) {
            console.log('msgs[1]', msgs[1])
            // if already running
            if (msgs[1].includes('already running')) {
              $('#toggle-rover-listener-btn')[0].checked = true
            } else {
              $('#toggle-rover-listener-btn')[0].checked = false
            }
          } else {
            if (msgs[0]) {
              $('#toggle-rover-listener-btn')[0].checked = false
            } else {
              $('#toggle-rover-listener-btn')[0].checked = true
            }
          }
        })
        // console.log('returnVals', returnVals)
      } else {
        appendToConsole(
          'Cannot turn rover listener on if not in rover mux channel!'
        )
      }
    } else {
      // closing rover listener
      requestTask('rover_listener', 0, '#toggle-rover-listener-btn', function (
        msgs
      ) {
        console.log('msgs[0]', msgs[0])
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
      })
    }
  })
})

// commands to change speed settings, get buffered serial messages
$(document).keydown(function (e) {
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
    case 85: // 'u' --> decrease max throttle
      lightUp('#max-throttle-decrease > button')
      maxSoftThrottle -= maxThrottleIncrement
      if (maxSoftThrottle < 0 ) {
        maxSoftThrottle = 0
      }
      $('#max-throttle-speed').text(maxSoftThrottle)
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
    case 74: // 'j' --> decrease max steering
      lightUp('#max-steering-decrease > button')
      maxSoftSteering -= maxSteeringIncrement
      if (maxSoftSteering < 0) {
        maxSoftSteering = 0
      }
      $('#max-steering-speed').text(maxSoftSteering)
      lastCmdSent = new Date().getTime()
      break

    case 66: // 'b' --> get buffered serial messages
      if (millisSince(lastCmdSent) > MCU_FEEDBACK_THROTTLE) {
        lightUp('button#show-buffered-rover-msgs')

        $.ajax({
          url: '/rover_drive',
          type: 'POST',
          data: {
            cmd: 'b'
          },
          success: function (response) {
            appendToConsole('cmd: ' + response.cmd)
            appendToConsole('feedback:\n' + response.feedback)
            if (response.error != 'None') {
              appendToConsole('error:\n' + response.error)
            }
            scrollToBottom()
          }
        })
        lastCmdSent = new Date().getTime()
      }
      break
    case 77: // 'm' --> enable motor control
      lightUp('button#enable-rover-motors')

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'm'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            enableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 78: // 'n' --> disable motor control
      lightUp('button#disable-rover-motors')

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'n'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            disableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 76: // 'l' --> list all commands
      lightUp('button#list-all-rover-cmds')

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'l'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            disableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 84: // 't' --> toggle listener proxy script
      if (millisSince(lastCmdSent) > LISTENER_TOGGLE_THROTTLE) {
        lightUp('button#toggle-rover-listener-btn')

        let request = ''

        if ($('#toggle-rover-listener').is(':checked')) {
          request = 'disable-rover-listener'
        } else {
          request = 'enable-rover-listener'
        }

        $.ajax({
          url: '/task_handler',
          type: 'POST',
          data: {
            cmd: request
          },
          success: function (response) {
            console.log(response)
            appendToConsole('cmd: ' + response.cmd)
            appendToConsole('output: ' + response.output)
            if (
              !response.output.includes('Failed') &&
              !response.output.includes('shutdown request') &&
              !response.output.includes('unavailable')
            ) {
              if ($('#toggle-rover-listener').is(':checked')) {
                disableRoverListenerBtn()
              } else {
                enableRoverListenerBtn()
              }
            }

            if (response.error != 'None') {
              appendToConsole('error:\n' + response.error)
            }
            scrollToBottom()
          }
        })
        lastCmdSent = new Date().getTime()
      }
      break
    case 81: // 'q' --> terminate (quit) listener script
      lightUp('button#terminate-listener-script')

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'q'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            disableRoverListenerBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    default:
      return // exit this handler for other keys
  }
  e.preventDefault() // prevent the default action (scroll / move caret)
})

// no throttling necessary as since keydown events are throttled
// those keys will not change color and the following code will only set it to it's default color
$(document).keyup(function (e) {
  switch (e.which) {
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

    case 77: // enable rover motors
      dim('button#enable-rover-motors')
      break
    case 78: // disable rover motors
      dim('button#disable-rover-motors')
      break
    case 66: // show buffered serial messages from the MCU
      dim('button#show-buffered-rover-msgs')
      break
    case 76: // list all rover cmds
      dim('button#list-all-rover-cmds')
      break
    case 84: // toggle rover listener proxy script
      dim('button#toggle-rover-listener-btn')
      break
    case 81: // terminate listener script (quit)
      dim('button#terminate-listener-script')
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

function gameLoop () {
  if (millisSince(lastCmdSent) > DRIVE_THROTTLE_TIME) {
    // 'a' --> rover turn left
    if (keyState[65]) {
      lightUp('#rover-left > button')
      if (steering < 0) {
        steering += 3*steeringIncrement
      } else {
        steering += steeringIncrement
      }
      if (steering > maxSoftSteering) {
        steering = maxSoftSteering
      }
      lastCmdSent = new Date().getTime()
    }
    // 'd' --> rover right
    else if (keyState[68]) {
      lightUp('#rover-right > button')

      if (steering > 0) {
        steering -= 3*steeringIncrement
      } else {
        steering -= steeringIncrement
      }
      if (steering < -maxSoftSteering) {
        steering = -maxSoftSteering
      }
      lastCmdSent = new Date().getTime()
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
    if (keyState[87]) {
      lightUp('#rover-up > button')
      if (throttle < 0) {
        throttle += 3*throttleIncrement
      } else {
        throttle += throttleIncrement
      }
      if (throttle > maxSoftThrottle) {
        throttle = maxSoftThrottle
      }
      lastCmdSent = new Date().getTime()
    }
    // 's' --> rover back
    else if (keyState[83]) {
      lightUp('#rover-down > button')
      if (throttle > 0) {
        throttle -= 3*throttleIncrement
      } else {
        throttle -= throttleIncrement
      }
      if (throttle < -maxSoftThrottle) {
        throttle = -maxSoftThrottle
      }
      lastCmdSent = new Date().getTime()
    }
    // decelerate
    else {
      if (throttle < 0) {
        throttle += throttleIncrement
      } else if (throttle > 0) {
        throttle -= throttleIncrement
      }
    }
    $('#throttle-speed').text(throttle)
    $('#steering-speed').text(steering)
  }
  setTimeout(gameLoop, 10)
}

gameLoop()
