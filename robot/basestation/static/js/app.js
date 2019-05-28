/* eslint-disable no-unused-lets */
$(document).ready(() => {
  const Site = {
    init () {
      // connect ros to the rosbridge websockets server
      initRosWeb()

      this.bindEventHandlers()
    },
    bindEventHandlers () {
      this.eventHandlers.forEach(eventHandler => this.bindEvent(eventHandler))
    },
    bindEvent (e) {
      let intervalId
      const binder = () => {
        $.getJSON(e.route, e.handler)
        intervalId = setInterval(() => {
          $.getJSON(e.route, e.handler)
        }, 100)
        return false
      }
      e.$el.bind(e.event, binder)
      e.$el.bind('mouseup', () => {
        clearInterval(intervalId)
      })
    },
    eventHandlers: [
      {
        $el: $('#btn_pitch_up'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_up',
        handler: data => {}
      },
      {
        $el: $('#btn_pitch_down'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_down',
        handler: data => {}
      },
      {
        $el: $('#btn_roll_left'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_left',
        handler: data => {}
      },
      {
        $el: $('#btn_roll_right'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_right',
        handler: data => {}
      },
      {
        $el: $('#btn_claw_open'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_open',
        handler: data => {}
      },
      {
        $el: $('#btn_claw_close'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_close',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_up'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_up',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_down'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_down',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_left'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_left',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_right'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_right',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_backward'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_back',
        handler: data => {}
      },
      {
        $el: $('#btn_arm_forward'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_forward',
        handler: data => {}
      },
      {
        $el: $('#btn_motor1_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_ccw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor1_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_cw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor2_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_ccw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor2_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_cw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor3_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_ccw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor3_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_cw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor4_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_ccw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor4_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_cw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor5_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_ccw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor5_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_cw',
        handler: data => {}
      },
      {
        $el: $('#btn_motor6_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_ccw',
        handler: data => {}
      },
      {
        $el: $('a#btn_motor6_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_cw',
        handler: data => {}
      },
      {
        $el: $('button#ping-button'),
        event: 'mouseup',
        route: '/ping_rover',
        handler: data => {
          console.log(data)
          pingRover(data.ping_msg, data.ros_msg)
        }
      }
    ]
  }

  Site.init()

  function isListenerOpen () {
    return (
      (window.location.pathname == '/rover' &&
        $('#toggle-rover-listener-btn')[0].checked == true) ||
      (window.location.pathname == '/' &&
        $('#toggle-arm-listener-btn')[0].checked == true) ||
      (window.location.pathname == '/science' &&
        $('#science-listener-btn')[0].checked == true)
      // pds Listener
    )
  }

  // select mux channel using mux_select service
  $('#mux-0').mouseup(function () {
    // Rover
    if (isListenerOpen()) {
      appendToConsole("Don't change the mux channel while a listener is open!")
    } else {
      requestMuxChannel('#mux-0', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })

  $('#mux-1').mouseup(function () {
    // Arm
    if (isListenerOpen()) {
      appendToConsole("Don't change the mux channel while a listener is open!")
    } else {
      requestMuxChannel('#mux-1', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })

  $('#mux-2').mouseup(function () {
    // Science
    if (isListenerOpen()) {
      appendToConsole("Don't change the mux channel while a listener is open!")
    } else {
      requestMuxChannel('#mux-2', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })

  $('#mux-3').mouseup(function () {
    // PDS
    if (isListenerOpen()) {
      appendToConsole("Don't change the mux channel while a listener is open!")
    } else {
      requestMuxChannel('#mux-3', function (msgs) {
        console.log('msgs', msgs)
      })
    }
  })

  $('#uart').mouseup(function () {
    $('#serial-type').text('uart')
  })

  $('#usb').mouseup(function () {
    $('#serial-type').text('usb')
  })

  // send serial command based on mux channel and current page
  // beware that if choosing a different mux channel than the current page,
  // commands will probably mess something up until this is done in a smart manner
  $('#send-serial-btn').mouseup(function () {
    // b
    let cmd = $('#serial-cmd-input').val()
    let buttonText = $('button#mux').text()
    if (buttonText.includes('Select Device Channel')) {
      appendToConsole(
        'Unable to send serial command. Try opening a mux channel.'
      )
    } else {
      // if the appropriate listener is open, send a command to it
      if (
        buttonText.includes('Rover') &&
        $('#toggle-rover-listener-btn')[0].checked == true
      ) {
        // sendRoverCommand(cmd) // rover commands not yet implemented
      } else if (
        buttonText.includes('Arm') &&
        $('#toggle-arm-listener-btn')[0].checked == true
      ) {
        sendArmCommand(cmd)
      } else if (buttonText.includes('Science')) {
        // science buttons unknown
        // sendScienceCommand(cmd) // science commands not yet implemented
      } else if (buttonText.includes('PDS')) {
        // pds buttons unknown
        // sendPdsCommand(cmd) // pds commands not yet implemented
      }
      // no listener is open, send generic request
      else if (!buttonText.includes('Select Device Channel')) {
        requestSerialCommand(cmd, function (msgs) {
          console.log(msgs)
          if (msgs[0]) {
            console.log('nice')
          } else {
            console.log('not nice')
          }
        })
      }
    }
  })

  $('#serial-cmd-input').on('keyup', function (e) {
    if (e.keyCode == 13) {
      // enter key
      // copy code from above
    }
  })
})
