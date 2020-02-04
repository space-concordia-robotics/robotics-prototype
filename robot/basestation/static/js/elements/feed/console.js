$(document).ready(() => {
  function pingDevice (device) {
    if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
      switch (device) {
        case 'Arm':
          sendRequest('Arm', 'ping', printErrToConsole)
          break
        case 'Rover':
          sendRequest('Rover', 'ping', printErrToConsole)
          break
        case 'Odroid':
        default:
          pingOdroid()
          break
      }
      lastCmdSent = new Date().getTime()
    }
  }

  function pingOdroid (timeoutVal = REQUEST_TIMEOUT) {
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
      error: function (jqXHR, textStatus, errorThrown) {
        console.log(errorThrown)
        if (errorThrown == 'timeout') {
          msg =
            'Odroid ping timeout after ' +
            timeoutVal / 1000 +
            ' seconds. ' +
            'Check if the websockets server is running. If not, there's either a network issue ' +
            'or the Odroid and possibly the whole rover has shut down unexpectedly.'
          appendToConsole(msg)
        } else {
          console.log('Error of type ' + errorThrown + 'occured')
        }
      },
      timeout: timeoutVal
    })
    lastCmdSent = new Date().getTime()
  }

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
})
