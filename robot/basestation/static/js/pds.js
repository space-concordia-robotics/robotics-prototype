const PING_THROTTLE_TIME = 1000
const PDS_REQUEST_TIMEOUT = 3000
let lastCmdSent = 0

function printCommandsList () {
  appendToConsole("'ctrl-alt-p': ping odroid")
  appendToConsole("'p': ping rover mcu")
  appendToConsole("'q': cut power to all motors")
  appendToConsole("'l': view key commands")
}

$(document).ready(function () {
  $('#list-all-cmds').on('click', function(event){
    event.preventDefault()
    printCommandsList()
  })

  $('#toggle-pds-listener-btn').on('click', function (event) {
    event.preventDefault()
    let serialType = $('#serial-type')
      .text()
      .trim()
    // click makes it checked during this time, so trying to enable
    if ($('#toggle-pds-listener-btn').is(':checked')) {
      if (
        $('button#mux')
          .text()
          .includes('PDS')
      ) {
      requestTask(
          'pds_listener',
          1,
          '#toggle-pds-listener-btn',
          function (msgs) {
            if (msgs[0]) {
              $('#toggle-pds-listener-btn')[0].checked = true
            } else {
              $('#toggle-pds-listener-btn')[0].checked = false
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
      // closing PDS listener
      requestTask(
        'pds_listener',
        0,
        '#toggle-pds-listener-btn',
        function (msgs) {
          console.log('msgs[0]', msgs[0])
          if (msgs.length == 2) {
            console.log('msgs[1]', msgs[1])
            if (msgs[1].includes('already running')) {
              $('#toggle-pds-listener-btn')[0].checked = true
            } else {
              $('#toggle-pds-listener-btn')[0].checked = false
            }
          } else {
            if (msgs[0]) {
              $('#toggle-pds-listener-btn')[0].checked = true
            } else {
              $('#toggle-pds-listener-btn')[0].checked = false
            }
          }
        },
        serialType
      )
    }
  })
  $('#toggle-auto-mode-btn').on('click', function (event) {
    event.preventDefault()
    if ($('#toggle-auto-mode-btn').is(':checked')){
      sendRequest("PDS", 'PDS T 1', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[0]) {
          $('#toggle-auto-mode-btn')[0].checked = true
        }
      }, PDS_REQUEST_TIMEOUT)
    } else {
      sendRequest("PDS", 'PDS T 0', function (msgs) {
        printErrToConsole(msgs)
        if (msgs[0]) {
          $('#toggle-auto-mode-btn')[0].checked = false
        }
      }, PDS_REQUEST_TIMEOUT)
    }
  })

  $('#cut-pds-power-button').on('click', function(event){
    event.preventDefault()
    sendPdsCommand('PDS S')
  })
  $('#provide-pds-power-button').on('click', function(event){
    event.preventDefault()
    sendPdsCommand('PDS A')
  })

  $('[id$=-power-btn]').on('click', function (event) {
    event.preventDefault()
    //if ($('button#mux').text().includes('PDS')) {
    let powerBtnID = this.id
    let num = this.id[1]
    let isPowered = !$(this.id).is(':checked')
    //console.log('PDS M ' + num + ' ' + ((isPowered) ? '1' : '0'))
    if ($('#m'+num+'-power-btn').is(':checked')){
        sendRequest("PDS", 'PDS M ' + num + ' ' + '1', function (msgs) {
          printErrToConsole(msgs)
          if (msgs[0]) {
            $('#m'+num+'-power-btn')[0].checked = true
          }
        }, PDS_REQUEST_TIMEOUT)
      } else {
        sendRequest("PDS", 'PDS M ' + num + ' ' + '0', function (msgs) {
          printErrToConsole(msgs)
          if (msgs[0]) {
            $('#m'+num+'-power-btn')[0].checked = false
          }
        }, PDS_REQUEST_TIMEOUT)
      }
    //} else {
      //appendToConsole('Cannot turn PDS listener on if not in PDS mux channel!')
    //}
  })

  $('#reset-general-flags-button').on('click', function (event) {
    event.preventDefault()
    sendPdsCommand('PDS G')
  })
  $('#reset-current-flags-button').on('click', function (event) {
    event.preventDefault()
    sendPdsCommand('PDS C')
  })

  $('#fan1-speed-btn').mouseup(function () {
    if ($('button#mux').text().includes('PDS')) {
      let num = this.id[3]
      let fanSpeed = $('#fan1-speed-input').val()
      let maxSpeed = 100
      if (
        parseInt(fanSpeed) >= 0 &&
        parseInt(fanSpeed) <= maxSpeed
      ) {
        //console.log('PDS F ' + num + ' ' + fanSpeed)
        sendPdsCommand('PDS F ' + num + ' ' + fanSpeed)
      }
    } else {
      appendToConsole('Cannot turn PDS listener on if not in PDS mux channel!')
    }
  })
  $('#fan2-speed-btn').mouseup(function () {
    if ($('button#mux').text().includes('PDS')) {
      let num = this.id[3]
      let fanSpeed = $('#fan2-speed-input').val()
      let maxSpeed = 100
      if (
        parseInt(fanSpeed) >= 0 &&
        parseInt(fanSpeed) <= maxSpeed
      ) {
        //console.log('PDS F ' + num + ' ' + fanSpeed)
        sendPdsCommand('PDS F ' + num + ' ' + fanSpeed)
      }
    } else {
      appendToConsole('Cannot turn PDS listener on if not in PDS mux channel!')
    }
  })
})

// KEYBOARD EVENTS
// odroid ping
document.addEventListener('keydown', function (event) {
  if (
    event.ctrlKey &&
    event.altKey &&
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    pingDevice('Odroid')
    lastCmdSent = new Date().getTime()
  }
})
// pds mcu ping
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyP' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    pingDevice('PDS')
    lastCmdSent = new Date().getTime()
  }
})
// print commands list
document.addEventListener('keydown', function (event) {
  if (
    event.code === 'KeyL' &&
    millisSince(lastCmdSent) > PING_THROTTLE_TIME
  ) {
    printCommandsList()
    lastCmdSent = new Date().getTime()
  }
})

// commands to change speed settings, get buffered serial messages
$(document).keydown(function (e) {
  if (!$('#servo-val').is(':focus')) {
    switch (e.which) {
      case 81: // 'q' --> cut all power
        sendPdsCommand('PDS S')
        break
/*
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

      case 85: // 'u' --> decrease max throttle
        lightUp('#max-throttle-decrease > button')
        maxSoftThrottle -= maxThrottleIncrement
        if (maxSoftThrottle < 0) {
          maxSoftThrottle = 0
        }
        $('#max-throttle-speed').text(maxSoftThrottle)
        lastCmdSent = new Date().getTime()
        break
*/
      default:
        return // exit this handler for other keys
    }
    e.preventDefault() // prevent the default action (scroll / move caret)
  }
})