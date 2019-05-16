let mockArmTable = false

// update console log
if (mockArmTable) {
  setInterval(mockArmTableLog, 1000)
}

// @TODO: fix implementation of odroid rx pub/sub to work on event triggers
// rather than through polling
// update odroid rx data every second
// setInterval(updateOdroidRx, 1000);

// for command thoughput limiting
const MANUAL_CONTROL_THROTTLE_TIME = 100
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
let lastCmdSent = 0

function printCommandsList() {
  appendToConsole("Welcome to the arm command page! Here are some commands:")
  appendToConsole("'p': ping")
  appendToConsole("'z': emergency stop all motors")
  appendToConsole("'o': reset memorized angle values")
  appendToConsole("'l': view key commands")
  appendToConsole("Keys 'w' to 'u': move motors 1-6 forwards")
  appendToConsole("Keys 's' to 'j': move motors 1-6 backwards")
}
printCommandsList()

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
  $('#toggle-arm-listener-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-arm-listener-btn').is(':checked')) {
      requestTask("arm_listener", 1, '#toggle-arm-listener-btn')
    } else { // closing arm listener
      requestTask("arm_listener", 0, '#toggle-arm-listener-btn')
    }
  })
  $('#toggle-arm-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-arm-stream-btn').is(':checked')) {
      if (requestTask("camera_stream", 1, '#toggle-arm-stream-btn')) {
        $('img#camera-feed')[0].src =
          'http://' + getRoverIP() + ':8090/?action=stream'
      } else { // failed to open stream
        $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
      }
    }
    else { // closing stream
      requestTask("camera_stream", 0, '#toggle-arm-stream-btn')
      $('img#camera-feed')[0].src = '../static/images/stream-offline.jpg'
    }
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
    $.ajax('/ping_rover', {
      success: function (data) {
        appendToConsole(data.ping_msg)
        if (!data.ros_msg.includes('Response')) {
          appendToConsole('No response from ROS ping_acknowledgment service')
        } else {
          appendToConsole(data.ros_msg)
        }
        scrollToBottom()
      },
      error: function () {
        console.log('An error occured')
      }
    })
    lastCmdSent = new Date().getTime()
  }
})

// ping arm MCU
document.addEventListener('keydown', function (event) {
  if (
    !$serialCmdInput.is(':focus') &&
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    $('button#ping-arm-mcu').css('background-color', 'rgb(255, 0, 0)')
    sendArmRequest('ping')
    lastCmdSent = new Date().getTime()
  }
})

// print commands list
document.addEventListener('keydown', function (event) {
  if (
    !$serialCmdInput.is(':focus') &&
    event.code === 'KeyL' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    $('button#list-all-cmds').css('background-color', 'rgb(255, 0, 0)')
    printCommandsList()
    lastCmdSent = new Date().getTime()
  }
})

if (mockArmTable) {
  // manual controls
  let $serialCmdInput = $('#serial-cmd-input')

  // motor 1
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyW') {
      toggleToManual()
      $('#click_btn_motor1_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m1-angle').text()

        if (currentAngle > -350) {
          // simulate motor angles
          $('#m1-angle').text(parseFloat(currentAngle) - 1)
          // simulate motor currents
          $('#m1-current').text('3.5')
        }
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyS') {
      toggleToManual()
      $('#click_btn_motor1_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m1-angle').text()

        if (currentAngle < 350) {
          $('#m1-angle').text(parseFloat(currentAngle) + 1)
          // simulate motor currents
          $('#m1-current').text('3.5')
        }
      }
    }
  })

  // motor 2
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyE') {
      toggleToManual()
      $('#click_btn_motor2_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m2-angle').text()

        if (currentAngle > -75) {
          $('#m2-angle').text(parseFloat(currentAngle) - 1)
          // simulate motor currents
          $('#m2-current').text('3.5')
        }
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyD') {
      toggleToManual()
      $('#click_btn_motor2_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m2-angle').text()

        if (currentAngle < 55) {
          $('#m2-angle').text(parseFloat(currentAngle) + 1)
          // simulate motor currents
          $('#m2-current').text('3.5')
        }
      }
    }
  })

  // motor 3
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyR') {
      toggleToManual()
      $('#click_btn_motor3_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m3-angle').text()

        if (currentAngle > -155) {
          $('#m3-angle').text(parseFloat(currentAngle) - 1)
        }
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyF') {
      toggleToManual()
      $('#click_btn_motor3_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m3-angle').text()

        if (currentAngle < 35) {
          $('#m3-angle').text(parseFloat(currentAngle) + 1)
        }
      }
    }
  })

  // motor 4
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyT') {
      toggleToManual()
      $('#click_btn_motor4_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m4-angle').text()

        if (currentAngle > -55) {
          $('#m4-angle').text(parseFloat(currentAngle) - 1)
        }
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyG') {
      toggleToManual()
      $('#click_btn_motor4_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        // no angle limits
        let currentAngle = $('#m4-angle').text()

        if (currentAngle < 40) {
          $('#m4-angle').text(parseFloat(currentAngle) + 1)
        }
      }
    }
  })

  // motor 5
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyY') {
      toggleToManual()
      $('#click_btn_motor5_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        // no angle limits
        let currentAngle = $('#m5-angle').text()
        $('#m5-angle').text(parseFloat(currentAngle) - 1)
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyH') {
      toggleToManual()
      $('#click_btn_motor5_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        // no angle limits
        let currentAngle = $('#m5-angle').text()
        $('#m5-angle').text(parseFloat(currentAngle) + 1)
      }
    }
  })

  // motor 6
  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyU') {
      toggleToManual()
      $('#click_btn_motor6_ccw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m6-angle').text()

        if (currentAngle > 0) {
          $('#m6-angle').text(parseFloat(currentAngle) - 1)
        }
      }
    }
  })

  document.addEventListener('keydown', function (event) {
    if (!$serialCmdInput.is(':focus') && event.code === 'KeyJ') {
      toggleToManual()
      $('#click_btn_motor6_cw > button').css(
        'background-color',
        'rgb(255, 0, 0)'
      )

      if (mockArmTable) {
        let currentAngle = $('#m6-angle').text()

        if (currentAngle < 75) {
          $('#m6-angle').text(parseFloat(currentAngle) + 1)
        }
      }
    }
  })
} else {
  // Implement game loop
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
    let $serialCmdInput = $('#serial-cmd-input')

    if (millisSince(lastCmdSent) > MANUAL_CONTROL_THROTTLE_TIME) {
      let budgeArray = ['~','~','~','~','~','~']
      let i=0
      let toBudge = false
      // 'w' --> m1 ccw
      if (!$serialCmdInput.is(':focus') && keyState[87]) {
        toggleToManual()
        $('#click_btn_motor1_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 's' --> m1 cw
      else if (!$serialCmdInput.is(':focus') && keyState[83]) {
        toggleToManual()
        $('#click_btn_motor1_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      i+=1
      // 'e' --> m2 ccw
      if (!$serialCmdInput.is(':focus') && keyState[69]) {
        toggleToManual()
        $('#click_btn_motor2_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 'd' --> m2 cw
      else if (!$serialCmdInput.is(':focus') && keyState[68]) {
        toggleToManual()
        $('#click_btn_motor2_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      i+=1
      // 'r' --> m3 ccw
      if (!$serialCmdInput.is(':focus') && keyState[82]) {
        toggleToManual()
        $('#click_btn_motor3_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 'f' --> m3 cw
      else if (!$serialCmdInput.is(':focus') && keyState[70]) {
        toggleToManual()
        $('#click_btn_motor3_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      i+=1
      // 't' --> m4 ccw
      if (!$serialCmdInput.is(':focus') && keyState[84]) {
        toggleToManual()
        $('#click_btn_motor4_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 'g' --> m4 cw
      else if (!$serialCmdInput.is(':focus') && keyState[71]) {
        toggleToManual()
        $('#click_btn_motor4_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      i+=1
      // 'y' --> m5 ccw
      if (!$serialCmdInput.is(':focus') && keyState[89]) {
        toggleToManual()
        $('#click_btn_motor5_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 'h' --> m5 cw
      else if (!$serialCmdInput.is(':focus') && keyState[72]) {
        toggleToManual()
        $('#click_btn_motor5_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      i+=1
      // 'u' --> m6 ccw
      if (!$serialCmdInput.is(':focus') && keyState[85]) {
        toggleToManual()
        $('#click_btn_motor6_ccw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='fwd'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }
      // 'j' --> m6 cw
      else if (!$serialCmdInput.is(':focus') && keyState[74]) {
        toggleToManual()
        $('#click_btn_motor6_cw > button').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        budgeArray[i]='back'
        toBudge = true
        lastCmdSent = new Date().getTime()
      }

      // 'z' --> stop all motors
      if (!$serialCmdInput.is(':focus') && keyState[90]) {
        $('button#stop-all-motors').css('background-color', 'rgb(255, 0, 0)')
        sendArmCommand('stop')
        lastCmdSent = new Date().getTime()
      }
      else if (toBudge) {
        let cmd = 'budge '
        for (var motor in budgeArray) {
          cmd += budgeArray[motor]
          if (motor != 5) {
            cmd += ' '
          }
        }
        sendArmCommand(cmd)
      }

      // 'o' --> reset angle values
      if (!$serialCmdInput.is(':focus') && keyState[79]) {
        $('button#reset-motor-angles').css('background-color', 'rgb(255, 0, 0)')
        sendArmCommand('reset')
        lastCmdSent = new Date().getTime()
      }

      // redraw/reposition your object here
      // also redraw/animate any objects not controlled by the user
    }

    setTimeout(gameLoop, 10)
  }
  gameLoop()
}

// In any case
// the following code just makes the buttons stop lighting up
// when the user stops pressing the respective key
let $serialCmdInput = $('#serial-cmd-input')

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyW') {
    $('#click_btn_motor1_ccw > button').css('background-color', 'rgb(74, 0, 0)')

    if (mockArmTable) {
      $('#m1-current').text('0.2')
    }
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyS') {
    $('#click_btn_motor1_cw > button').css('background-color', 'rgb(74, 0, 0)')
    $('#m1-current').text('0.2')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyE') {
    $('#click_btn_motor2_ccw > button').css('background-color', 'rgb(74, 0, 0)')
    $('#m2-current').text('0.2')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyD') {
    $('#click_btn_motor2_cw > button').css('background-color', 'rgb(74, 0, 0)')
    $('#m2-current').text('0.2')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyR') {
    $('#click_btn_motor3_ccw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyF') {
    $('#click_btn_motor3_cw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyT') {
    $('#click_btn_motor4_ccw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyG') {
    $('#click_btn_motor4_cw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyY') {
    $('#click_btn_motor5_ccw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyH') {
    $('#click_btn_motor5_cw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyU') {
    $('#click_btn_motor6_ccw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyJ') {
    $('#click_btn_motor6_cw > button').css('background-color', 'rgb(74, 0, 0)')
  }
})

// EXTRA CONTROLS
document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyP') {
    $('button#ping-arm-mcu').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyZ') {
    $('button#stop-all-motors').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyO') {
    $('button#reset-motor-angles').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyL') {
    $('button#list-all-cmds').css('background-color', 'rgb(74, 0, 0)')
  }
})

document.addEventListener('keyup', function (event) {
  if (!$serialCmdInput.is(':focus') && event.code === 'KeyA') {
    $('button#show-buffered-msgs').css('background-color', 'rgb(74, 0, 0)')
  }
})
