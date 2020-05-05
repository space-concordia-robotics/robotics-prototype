$(document).ready(function () {

  /* science commands */

  // setup a client for the science_request service
  science_request_client = new ROSLIB.Service({
    ros: ros,
    name: 'science_request',
    serviceType: 'ScienceRequest'
  })

  // setup a subscriber for the arm_joint_states topic
  science_data_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'science_feedback',
    messageType: 'std_msgs/String'
  })

  science_data_listener.subscribe(function (message) {
    appendToConsole(message.data, true, false)
    // isActivated, drillDirection, elevatorDirection
    let dataKeyValues = message.data.replace('Science data:', '').split(',')
    let keys = dataKeyValues.map(i => {
      return i.split(':')[0]
    })
    let values = dataKeyValues.map(i => {
      return i.split(':')[1]
    })

    // isActivated
    if (values[0] == '0') {
      $('#activate-science-btn')[0].checked = false
    } else if (values[0] == '1') {
      $('#activate-science-btn')[0].checked = true
    }

    // drillDirection
    if (values[1] == '0') {
      lightUp('#cw-btn')
      greyOut('#ccw-btn')
    } else if (values[1] == '1') {
      lightUp('#ccw-btn')
      greyOut('#cw-btn')
    }

    // elevatorDirection
    if (values[2] == '0') {
      lightUp('#elevator-up-btn')
      greyOut('#elevator-down-btn')
    } else if (values[2] == '1') {
      lightUp('#elevator-down-btn')
      greyOut('#elevator-up-btn')
    }


    if (values[13]) {
      $('#drill-rpm').val(values[13])
    }
    // drillInUse
    if (values[14] == '0') {
      lightUp('#drill-stop-btn')
      greyOut('#set-speed-go-btn')
      greyOut('#set-time-go-btn')
      greyOut('#drill-max-speed-go-btn')
    } else if (values[14] == '1') {
      greyOut('#drill-stop-btn')
    }

    // elevatorInUse
    if (values[15] == '0') {
      lightUp('#elevator-stop-btn')
      greyOut('#set-feed-go-btn')
      greyOut('#set-distance-go-btn')
      greyOut('#elevator-max-speed-go-btn')
    } else if (values[14] == '1') {
      greyOut('#elevator-stop-btn')
    }

    // tcwstepDone
    if (values[16] == '1') {
      if (millisSince(lastRotate) > ROTATE_TIMEOUT) {
        rotateNeg()
        lastRotate = Date.now()
      }
    }

    // tccwstepDone
    if (values[17] == '1') {
      if (millisSince(lastRotate) > ROTATE_TIMEOUT) {
        rotatePos()
        lastRotate = Date.now()
      }
    }

    // elevatorFeedPercent
    if (values[18]) {
      $('#elevator-feed-percent').val(values[18])
    }

    // currentTablePosition
    if (values[19]) {
      $('#table-position').val(values[19])
    }
  })

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
          SCIENCE_LISTENER_TASK,
          STATUS_START,
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
      requestTask(SCIENCE_LISTENER_TASK,
          STATUS_STOP, function (
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
