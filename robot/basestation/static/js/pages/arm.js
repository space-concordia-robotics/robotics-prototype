// @TODO: fix implementation of odroid rx pub/sub to work on event triggers
// rather than through polling
// update odroid rx data every second
// setInterval(updateOdroidRx, 1000);

// for command thoughput limiting
const MANUAL_CONTROL_THROTTLE_TIME = 100
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
let lastCmdSent = 0

function printCommandsList () {
  appendToConsole("'ctrl-alt-p': ping odroid")
  appendToConsole("'p': ping arm mcu")
  appendToConsole("'q': emergency stop all motors")
  appendToConsole("'o': reset memorized angle values")
  appendToConsole("'l': view key commands")
  appendToConsole("Keys 'w' to 'u': move motors 1-6 forwards")
  appendToConsole("Keys 's' to 'j': move motors 1-6 backwards")
}

function manualControl () {
  var armsOFF = $('#ArmcontrolsOFF')[0]
  var armsON = $('#ArmcontrolsON')[0]

  if (armsOFF.style.display === 'none') {
    armsOFF.style.display = 'block'
    armsON.style.display = 'none'
  } else {
    armsOFF.style.display = 'none'
    armsON.style.display = 'block'
    armsON.style.borderRadius = '0'
  }
}

function toggleToManual () {
  if (!$('#manual-control-btn')[0].checked) {
    $('#manual-control-btn').click()
  }
}

$(document).ready(function () {
  $('#ping-odroid').on('click', function (event) {
    event.preventDefault()
    if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
      pingDevice('Odroid')
      lastCmdSent = new Date().getTime()
    }
  })

  $('#toggle-arm-listener-btn').on('click', function (event) {
    event.preventDefault()
    let serialType = $('#serial-type')
      .text()
      .trim()
    if (
      $('#serialType')
        .text()
        .includes('Serial')
    ) {
      appendToConsole('Select a serial type!')
    }
    // click makes it checked during this time, so trying to enable
    else if ($('#toggle-arm-listener-btn').is(':checked')) {
      // validate UART mode options are correct, let pass if USB mode selected
      if (
        ($('button#mux')
          .text()
          .includes('Arm') &&
          serialType == 'uart') ||
        serialType == 'usb'
      ) {
        requestTask(
          ARM_LISTENER_TASK,
          STATUS_START,
          function (msgs) {
            printErrToConsole(msgs)
            if (msgs[0]) {
              $('#toggle-arm-listener-btn')[0].checked = true
            } else {
              $('#toggle-arm-listener-btn')[0].checked = false
            }
          },
          serialType
        )
      } else {
        appendToConsole(
          'UART MODE: Cannot turn arm listener on if not in arm mux channel!'
        )
      }
    } else {
      // closing arm listener
      requestTask(ARM_LISTENER_TASK, 
          STATUS_STOP, 
          function (
        msgs
      ) {
        printErrToConsole(msgs)
        if (msgs.length == 2) {
          if (msgs[1].includes('already running')) {
            $('#toggle-arm-listener-btn')[0].checked = true
          } else {
            $('#toggle-arm-listener-btn')[0].checked = false
          }
        } else {
          if (msgs[0]) {
            $('#toggle-arm-listener-btn')[0].checked = true
          } else {
            $('#toggle-arm-listener-btn')[0].checked = false
          }
        }
      })
    }
  })

  $('#arm-speed-multiplier-btn').mouseup(function () {
    let multiplier = $('#arm-speed-multiplier-input').val()
    let maxMultiplier = 5.0
    if (
      parseFloat(multiplier) >= 0 &&
      parseFloat(multiplier) <= maxMultiplier
    ) {
      let cmd = 'armspeed ' + multiplier
      sendRequest('Arm', cmd, printErrToConsole)
    }
  })

  $('#arm-speed-multiplier-input').on('keyup', function (e) {
    if (e.keyCode == 13) {
      // enter key
      let multiplier = $('#arm-speed-multiplier-input').val()
      let maxMultiplier = 5.0
      if (
        parseFloat(multiplier) >= 0 &&
        parseFloat(multiplier) <= maxMultiplier
      ) {
        let cmd = 'armspeed ' + multiplier
        sendRequest('Arm', cmd, printErrToConsole)
      }
    }
  })

  $('[id$=-closed-loop-btn]').on('click', function (event) {
    event.preventDefault()
    num = this.id[1]
    isOpen = !$(this.id).is(':checked')
    armReq = function (msgs) {
      if (msgs[0]) {
        $(this.id)[0].checked = !isOpen
      } else {
        $(this.id)[0].checked = isOpen
      }
    }
    sendRequest(
      'Arm',
      'motor ' + num + ' loop ' + (isOpen ? 'open' : 'closed'),
      armReq
    )
  })
})

// KEYBOARD EVENTS
// arm mcu ping
document.addEventListener('keydown', function (event) {
  if (
    !$('#serial-command-input').is(':focus') &&
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    pingDevice('Arm')
    lastCmdSent = new Date().getTime()
  }
})

// Implement game loop
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

// Button coloring
const YELLOW = 'rgb(255, 249, 148)'
const GREEN = 'rgb(61, 127, 127)'

function gameLoop () {
  if (millisSince(lastCmdSent) > MANUAL_CONTROL_THROTTLE_TIME) {
    let budgeArray = ['~', '~', '~', '~', '~', '~']
    let i = 0
    let toBudge = false
    // 'w' --> m1 ccw
    if (keyState[87]) {
      toggleToManual()
      $('#click_btn_motor1_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 's' --> m1 cw
    else if (keyState[83]) {
      toggleToManual()
      $('#click_btn_motor1_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    i += 1
    // 'e' --> m2 ccw
    if (keyState[69]) {
      toggleToManual()
      $('#click_btn_motor2_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'd' --> m2 cw
    else if (keyState[68]) {
      toggleToManual()
      $('#click_btn_motor2_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    i += 1
    // 'r' --> m3 ccw
    if (keyState[82]) {
      toggleToManual()
      $('#click_btn_motor3_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'f' --> m3 cw
    else if (keyState[70]) {
      toggleToManual()
      $('#click_btn_motor3_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    i += 1
    // 't' --> m4 ccw
    if (keyState[84]) {
      toggleToManual()
      $('#click_btn_motor4_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'g' --> m4 cw
    else if (keyState[71]) {
      toggleToManual()
      $('#click_btn_motor4_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    i += 1
    // 'y' --> m5 ccw
    if (keyState[89]) {
      toggleToManual()
      $('#click_btn_motor5_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'h' --> m5 cw
    else if (keyState[72]) {
      toggleToManual()
      $('#click_btn_motor5_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    i += 1
    // 'u' --> m6 ccw
    if (keyState[85]) {
      toggleToManual()
      $('#click_btn_motor6_ccw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'fwd'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'j' --> m6 cw
    else if (keyState[74]) {
      toggleToManual()
      $('#click_btn_motor6_cw > button').css(
        'background-color',
        YELLOW
      )

      budgeArray[i] = 'back'
      toBudge = true
      lastCmdSent = new Date().getTime()
    }
    // 'q' --> stop all motors
    if (keyState[81]) {
      sendArmCommand('stop')
      lastCmdSent = new Date().getTime()
    } else if (toBudge) {
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
    if (keyState[79]) {
      sendArmCommand('reset')
      lastCmdSent = new Date().getTime()
    }

    // numpad8 --> ARM UP
    if (keyState[104]) {
      console.log('ARM UP')
      lightUp('#arm-up-btn > button')
    }

    // numpad4 --> ARM LEFT
    if (keyState[100]) {
      console.log('ARM LEFT')
      lightUp('#arm-left-btn > button')
    }

    // numpad6 --> ARM RIGHT
    if (keyState[102]) {
      console.log('ARM RIGHT')
      lightUp('#arm-right-btn > button')
    }

    // numpad5 --> ARM DOWN
    if (keyState[101]) {
      console.log('ARM DOWN')
      lightUp('#arm-down-btn > button')
    }

    // numpad1 --> ARM BACK
    if (keyState[97]) {
      console.log('ARM BACK')
      lightUp('#arm-back-btn > button')
    }

    // numpad3 --> ARM FWD
    if (keyState[99]) {
      console.log('ARM FWD')
      lightUp('#arm-fwd-btn > button')
    }
    // redraw/reposition your object here
    // also redraw/animate any objects not controlled by the user
  }

  setTimeout(gameLoop, 10)
}
gameLoop()

// In any case
// the following code just makes the buttons stop lighting up
// when the user stops pressing the respective key

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyW') {
    $('#click_btn_motor1_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyS') {
    $('#click_btn_motor1_cw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyE') {
    $('#click_btn_motor2_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyD') {
    $('#click_btn_motor2_cw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyR') {
    $('#click_btn_motor3_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyF') {
    $('#click_btn_motor3_cw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyT') {
    $('#click_btn_motor4_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyG') {
    $('#click_btn_motor4_cw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyY') {
    $('#click_btn_motor5_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyH') {
    $('#click_btn_motor5_cw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyU') {
    $('#click_btn_motor6_ccw > button').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyJ') {
    $('#click_btn_motor6_cw > button').css('background-color', GREEN)
  }
})

// EXTRA CONTROLS
document.addEventListener('keyup', function (event) {
  if (event.code === 'KeyA') {
    $('button#show-buffered-msgs').css('background-color', GREEN)
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad8') {
    dim('#arm-up-btn > button')
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad4') {
    dim('#arm-left-btn > button')
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad6') {
    dim('#arm-right-btn > button')
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad5') {
    dim('#arm-down-btn > button')
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad1') {
    dim('#arm-back-btn > button')
  }
})

document.addEventListener('keyup', function (event) {
  if (event.code === 'Numpad3') {
    dim('#arm-fwd-btn > button')
  }
})
