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

  // select mux channel using mux_select service
  $('#mux-0').mouseup(function () { // Arm
    requestMuxChannel('#mux-0')
  })

  $('#mux-1').mouseup(function () { // Rover
    requestMuxChannel('#mux-1')
  })

  $('#mux-2').mouseup(function () { // Science
    requestMuxChannel('#mux-2')
  })

  $('#mux-3').mouseup(function () { // PDS // not lidar anymore
    requestMuxChannel('#mux-3')
  })

  // send serial command based on mux channel and current page
  // beware that if choosing a different mux channel than the current page,
  // commands will probably mess something up until this is done in a smart manner
  $('#send-serial-btn').mouseup(function () {
    // b
    let cmd = $('#serial-cmd-input').val()
    let buttonText = $('button#mux').text()
    if (buttonText.includes('Rover') && window.location.pathname=='/rover') {
      // sendRoverCommand(cmd) // rover commands not yet implemented
    } else if (buttonText.includes('Arm') && window.location.pathname=='/') {
      sendArmCommand(cmd)
    }
    // this implementation is tricky to think about... save it for later
    /*
    if (buttonText.includes('Rover')) {
      sendArmCommand(cmd)
    } else if (buttonText.includes('Arm')) {
      // sendRoverCommand(cmd) // rover commands not yet implemented
    } else if (buttonText.includes('Science')) {
      // sendScienceCommand(cmd) // science commands not yet implemented
    } else if (buttonText.includes('PDS')) {
      // sendPdsCommand(cmd) // pds commands not yet implemented
    }
    else appendToConsole('Unable to send serial command. Mux channel ')
    */
  })

  $('#serial-cmd-input').on('keyup', function (e) {
    if (e.keyCode == 13) { // enter key
      // copy code from above
    }
  })

})
