PING_THROTTLE_TIME = 1000
ERROR_SOUND_PATH = '../../static/sound/error.wav'

/* eslint-disable no-unused-lets */
$(document).ready(() => {
  // @TODO: fix this hacky workaround (refer to issue #142)
  // https://github.com/space-concordia-robotics/robotics-prototype/issues/142
  lastCmdSent = 0
  // end of said hacky workaround

  const Site = {
    init () {
      // connect ros to the rosbridge websockets server
      initRosWeb()

      if (getCookie('serialType')) {
        appendToConsole('Found cookie!')
        $('#serial-type').text(getCookie('serialType'))
      }

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
})

function printErrToConsole (msg) {
  if (!msg[0]) appendToConsole(msg[1])
}

function pingDevice (device) {
  if (millisSince(lastCmdSent) > PING_THROTTLE_TIME) {
    switch (device) {
      case 'Arm':
        sendRequest('Arm', 'ping', printErrToConsole)
        break
      case 'Rover':
        sendRequest('Rover', 'ping', printErrToConsole)
        break
      case 'PDS':
        sendRequest('PDS', 'PDS ping', printErrToConsole)
        break
      case 'Science':
        sendRequest('Science', 'ping', printErrToConsole)
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
          "Check if the websockets server is running. If not, there's either a network issue " +
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

/*
plays an error sound once -> warning will not play if the document has not been interacted with yet
*/
function errorSound (){
  new Audio(ERROR_SOUND_PATH).play()
}
