// for command thoughput limiting
const DRIVE_THROTTLE_TIME = 75
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
// constants for speed setting limits (absolute max: 45)
const MAX_THROTTLE_SPEED = 45
const MAX_STEERING_SPEED = 39
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

function printCommandsList () {
  appendToConsole("'ctrl-alt-p': ping odroid")
  appendToConsole("'p': ping arm mcu")
  appendToConsole("'q': emergency stop all motors")
  appendToConsole("'l': view key commands")
  //appendToConsole("Keys 'w' to 'u': move motors 1-6 forwards")
  //appendToConsole("Keys 's' to 'j': move motors 1-6 backwards")
}

// Manual control
function manualControl () {
  var a = document.getElementById('ArmcontrolsOFF')
  var b = document.getElementById('ArmcontrolsON')

  if (a.style.display === 'none') {
    a.style.display = 'block'
    b.style.display = 'none'
  } else {
    a.style.display = 'none'
    b.style.display = 'block'
    b.style.borderRadius = '0'
  }
}

function toggleToManual () {
  if (!$('#manual-control-btn')[0].checked) {
    $('#manual-control-btn').click()
  }
}

$(document).ready(function () {

  $('#ping-odroid').on('click', function (event) {
    if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
      appendToConsole('pinging odroid')
      $.ajax('/ping_rover', {
        success: function (data) {
          appendToConsole(data.ping_msg)
          if (!data.ros_msg.includes('Response')) {
            appendToConsole('No response from ROS ping_acknowledgment service')
          } else {
            appendToConsole(data.ros_msg)
          }
        },
        error: function () {
          console.log('An error occured')
        }
      })
      lastCmdSent = new Date().getTime()
    }
  })
  $('#ping-rover-mcu').on('click', function (event) {
    event.preventDefault()
    if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
      sendRoverRequest('ping', function (msgs) {})
      lastCmdSent = new Date().getTime()
    }
  })

  $('#reboot-button').on('click', function (event) {
    event.preventDefault()
    sendRoverCommand('reboot')
  })

  $('#activate-rover-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#rover-listener-btn').is(':checked')) {
      appendToConsole('Rover listener not yet activated!')
    } else if ($('#activate-rover-btn').is(':checked')) {
      sendScienceRequest('activate', function (msgs) {
        console.log('msgs', msgs)
      })
    } else {
      // 'deactivated' needs to be handled differently since it takes 45 secconds
      sendRoverRequest('deactivate', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })
  $('#toggle-rover-listener-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-rover-listener-btn').is(':checked')) {
      let serialType = 'uart' // in a perfect world this is controlled in the gui
      if ( $('button#mux').text().includes('Rover') || serialType == 'usb') {
        requestTask('rover_listener', 1, '#toggle-rover-listener-btn', function (msgs) {
          if (msgs[0]) {
            $('#toggle-rover-listener-btn')[0].checked = true
            } else {
              $('#toggle-rover-listener-btn')[0].checked = false
            }
          }, serialType)
        } else {
        appendToConsole(
          'Cannot turn rover listener on if not in rover mux channel!')
      }
    } else {
      // closing rover listener
      requestTask('rover_listener', 0, '#toggle-rover-listener-btn', function (msgs) {
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
  $('#front-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#front-camera-stream-btn').is(':checked')) {
      requestTask('camera_stream', 1, '#front-camera-stream-btn', function (msgs) {
        if (msgs[0]) {
          $('img#camera-feed')[0].src =
            'http://' + getRoverIP() + ':8090/?action=stream'
        } else {
          // failed to open stream
          $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
        }
      }, '/dev/ttyFrontCam')
    } else {
      requestTask('camera_stream', 0, '#front-camera-stream-btn', function (msgs) {
        if (msgs[0]) {
          // succeeded to close stream
          $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
        } else {
          // failed to close stream
          $('img#camera-feed')[0].src =
            'http://' + getRoverIP() + ':8090/?action=stream'
        }
      })
    }
  })
  $('#rear-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#rear-camera-stream-btn').is(':checked')) {
      requestTask('camera_stream', 1, '#rear-camera-stream-btn', function (msgs) {
        if (msgs[0]) {
          $('img#camera-feed')[0].src =
            'http://' + getRoverIP() + ':8090/?action=stream'
        } else {
          // failed to open stream
          $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
        }
      }, '/dev/ttyRearCam')
    } else {
      requestTask('camera_stream', 0, '#rear-camera-stream-btn', function (msgs) {
        if (msgs[0]) {
          // succeeded to close stream
          $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
        } else {
          // failed to close stream
          $('img#camera-feed')[0].src =
            'http://' + getRoverIP() + ':8090/?action=stream'
        }
      })
    }
  })

  $('#m1-closed-loop-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#m1-closed-loop-btn').is(':checked')) {
      sendArmRequest('motor 1 loop closed', function (msgs) {
        if (msgs[0]) {
          $('#m1-closed-loop-btn')[0].checked = true
          //console.log('trueee')
        } else {
          $('#m1-closed-loop-btn')[0].checked = false
          //console.log('falseeee')
        }
      })
    } else {
      sendArmRequest('motor 1 loop open', function (msgs) {
        if (msgs[0]) {
          $('#m1-closed-loop-btn')[0].checked = false
          //console.log('falseeee')
        } else {
          $('#m1-closed-loop-btn')[0].checked = true
          //console.log('trueee')
        }
      })
    }
  })

  $('#send-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if ( !($('#antenna-latitude-input').val()) ) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if ( !($('#antenna-longitude-input').val()) ) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if ( !($('#antenna-start-dir-input').val()) ) {
      appendToConsole('bearing field empty!')
      goodInput = false
    }
    if (goodInput) {
      let initialLatitude = $('#antenna-latitude-input').val()
      let initialLongitude = $('#antenna-longitude-input').val()
      let initialBearing = $('#antenna-start-dir-input').val()
      antenna_latitude.set( parseFloat(initialLatitude) );
      antenna_longitude.set( parseFloat(initialLongitude) );
      antenna_start_dir.set( parseFloat(initialBearing) );
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
    if ( !($('#goal-latitude-input').val()) ) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if ( !($('#goal-longitude-input').val()) ) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (goodInput) {
      let desiredLatitude = $('#goal-latitude-input').val()
      let desiredLongitude = $('#goal-longitude-input').val()
      goal_latitude.set( parseFloat(desiredLatitude) );
      goal_longitude.set( parseFloat(desiredLongitude) );
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
// rover ping
document.addEventListener('keydown', function (event) {
  if (
    event.ctrlKey &&
    event.altKey &&
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    appendToConsole('pinging odroid')
    $.ajax('/ping_rover', {
      success: function (data) {
        appendToConsole(data.ping_msg)
        if (!data.ros_msg.includes('Response')) {
          appendToConsole('No response from ROS ping_acknowledgment service')
        } else {
          appendToConsole(data.ros_msg)
        }
      },
      error: function () {
        console.log('An error occured')
      }
    })
    lastCmdSent = new Date().getTime()
  }
})
// rover mcu ping
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    sendRoverRequest('ping', function (msgs) {})
    lastCmdSent = new Date().getTime()
  }
})
// print commands list
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyL' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    $('button#list-all-cmds').css('background-color', 'rgb(255, 0, 0)')
    printCommandsList()
    lastCmdSent = new Date().getTime()
  }
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
sentZero = true
function gameLoop () {
  if (millisSince(lastCmdSent) > DRIVE_THROTTLE_TIME) {
    // 'd' --> rover right
    if (keyState[68]) {
      lightUp('#rover-right > button')
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
    // 'a' --> rover turn left
    else if (keyState[65]) {
      lightUp('#rover-left > button')

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
    $('#steering-speed').text(steering)
    if (throttle == 0 && sentZero) {
      ; // do nothing
    } else {
      $('#throttle-speed').text(throttle)
      let cmd = throttle.toString()+':'+steering.toString()
      sendRoverCommand(cmd)
      lastCmdSent = new Date().getTime()
      if (throttle != 0) {
        sentZero = false
      } else {
        sentZero = true
      }
    }
  }
  setTimeout(gameLoop, 10)
}

gameLoop()
