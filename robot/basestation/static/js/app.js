function requestMuxChannel(elemID) {
  let dev = elemID[elemID.length -1]
  let request = new ROSLIB.ServiceRequest({ device : dev })
  let sentTime = new Date().getTime()

  appendToConsole('Sending request to switch to channel ' + $('a'+elemID).text())

  mux_select_client.callService(request, function(result){
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response.slice(0, result.response.length-1)
    if (msg.includes('failed') || msg.includes('ERROR')) { // how to account for a lack of response?
      appendToConsole('Request failed. Received \"' + msg + '"')
    }
    else {
      $('button#mux').text('Device ' + $('a'+elemID).text())
      appendToConsole('Received \"' + msg + '\" with ' + latency.toString() + ' ms latency')
    }
    scrollToBottom()
  })
}

/* eslint-disable no-unused-lets */
$(document).ready(() => {
  const Site = {
    init () {
      // connect ros to the rosbridge websockets server
      let ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
      })

      ros.on('connection', function() {
        appendToConsole('Connected to websocket server.')
      })

      ros.on('error', function(error) {
        appendToConsole('Error connecting to websocket server: ', error)
      })

      ros.on('close', function() {
        appendToConsole('Connection to websocket server closed.')
      })

      // setup a client for the mux_select_service
      mux_select_client = new ROSLIB.Service({
        ros : ros, name : 'mux_select', serviceType : 'SelectMux'
      })

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

  $('#mux-0').mouseup(function () {
    requestMuxChannel('#mux-0')
  })

  $('#mux-1').mouseup(function () {
    requestMuxChannel('#mux-1')
  })

  $('#mux-2').mouseup(function () {
    requestMuxChannel('#mux-2')
  })

  $('#mux-3').mouseup(function () {
    requestMuxChannel('#mux-3')
  })

  $('#send-serial-btn').mouseup(function () {
    let cmd = $('#serial-cmd-input').val()

    $.ajax({
      url: '/serial_cmd',
      type: 'POST',
      data: {
        cmd: cmd
      },
      success: function (response) {
        if (!response.output.includes('Response')) {
          appendToConsole(response.output)
          appendToConsole('No response from serial_cmd ROS service\n')
        } else {
          appendToConsole(response.output)
          clearSerialCmd()
        }
        scrollToBottom()
      }
    })
  })

  $('#serial-cmd-input').on('keyup', function (e) {
    // enter key
    if (e.keyCode == 13) {
      let cmd = $('#serial-cmd-input').val()

      $.ajax({
        url: '/serial_cmd',
        type: 'POST',
        data: {
          cmd: cmd
        },
        success: function (response) {
          if (!response.output.includes('Response')) {
            appendToConsole(response.output)
            appendToConsole('No response from serial_cmd ROS service\n')
          } else {
            appendToConsole(response.output)
            clearSerialCmd()
          }
          scrollToBottom()
        }
      })
    }
  })
})
