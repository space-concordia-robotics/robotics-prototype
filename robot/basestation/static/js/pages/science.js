$(document).ready(function () {
  // MCU ping
  $('#ping-science-mcu').on('click', function (event) {
    event.preventDefault()
    sendRequest('Science', 'ping', printErrToConsole)
  })

  $('#ping-odroid').on('click', function (event) {
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
  })

  // ROS related stuff
  $('#science-listener-btn').on('click', function (event) {
    event.preventDefault()
    let serialType = $('#serial-type')
      .text()
      .trim()
    // click makes it checked during this time, so trying to enable
    if ($('#science-listener-btn').is(':checked')) {
      if (
        $('#serial-type')
          .text()
          .includes('Serial')
      ) {
        appendToConsole('Select a serial type!')
      }
      // validate UART mode options are correct, let pass if USB mode selected
      else if (
        ($('button#mux')
          .text()
          .includes('Science') &&
          serialType == 'uart') ||
        serialType == 'usb'
      ) {
        requestTask(
          'science_listener',
          1,
          '#science-listener-btn',
          function (msgs) {
            console.log(msgs)
            if (msgs.length == 2) {
              if (msgs[1].includes('already running') || msgs[0] == true) {
                $('#science-listener-btn')[0].checked = true
              } else {
                $('#science-listener-btn')[0].checked = false
              }
            }
          },
          serialType
        )
      } else {
        appendToConsole(
          'UART MODE: Cannot turn science listener on if not in science mux channel!'
        )
      }
    } else {
      // closing arm listener
      requestTask('science_listener', 0, '#science-listener-btn', function (
        msgs
      ) {
        console.log('msgs[0]', msgs[0])
        if (msgs.length == 2) {
          console.log('msgs[1]', msgs[1])
          if (msgs[1].includes('already running')) {
            $('#science-listener-btn')[0].checked = true
          } else {
            $('#science-listener-btn')[0].checked = false
          }
        } else {
          if (msgs[0]) {
            $('#science-listener-btn')[0].checked = true
          } else {
            $('#science-listener-btn')[0].checked = false
          }
        }
      })
    }
  })

  $('#activate-science-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#science-listener-btn').is(':checked')) {
      appendToConsole('Science listener not yet activated!')
    } else if ($('#activate-science-btn').is(':checked')) {
      sendRequest('Science', 'activate', function (msgs) {
        console.log('msgs', msgs)
      })
    } else {
      // 'deactivated' needs to be handled differently since it takes 45 secconds
      sendRequest('Science', 'stop', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })
})
