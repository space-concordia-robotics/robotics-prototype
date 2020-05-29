$(document).ready(() => {
  function pingDevice (device) {
    if (canSendCommand(PING_THROTTLE_TIME)) {
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
      setTimeSinceCommand()
    }
  }

  function pingOdroid (timeoutVal = REQUEST_TIMEOUT) {
    if (canSendCommand(PING_THROTTLE_TIME)) {
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
              'Check if the websockets server is running. If not, there\'s either a network issue ' +
              'or the Odroid and possibly the whole rover has shut down unexpectedly.'
            appendToConsole(msg)
          } else {
            console.log('Error of type ' + errorThrown + 'occured')
          }
        },
        timeout: timeoutVal
      })
    }
    setTimeSinceCommand()
  }

  document.addEventListener('keydown', function (event) {
    if (
      event.ctrlKey &&
      event.altKey &&
      event.code === 'KeyP' &&
      !$('#servo-val').is(':focus')
    ) {
      pingDevice('Odroid')
    }
  })

  // print commands list
  document.addEventListener('keydown', function (event) {
    if (
      event.code === 'KeyL' &&
      !$('#servo-val').is(':focus')
    ) {
      $('button#list-all-cmds').css('background-color', 'rgb(255, 0, 0)')
      printCommandsList()
    }
  })

  document.addEventListener('keyup', function (event) {
    if (event.code === 'KeyL') {
      $('button#list-all-cmds').css('background-color', 'rgb(68, 91, 123)')
    }
  })
})
