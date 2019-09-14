/* eslint-disable no-unused-lets */
$(document).ready(() => {
  const Site = {
    init () {
      // connect ros to the rosbridge websockets server
      initRosWeb()

      if (getCookie('serialType')) {
        appendToConsole('Found cookie!')
        $('#serial-type').text(getCookie('serialType'))
      }

      // only for ARM and ROVER (for now)
      if (window.location.pathname != '/science') {
        $('#front-camera-stream-btn').on('click', function (event) {
          event.preventDefault()
          // click makes it checked during this time, so trying to enable
          if ($('#front-camera-stream-btn').is(':checked')) {
            requestTask(
              'camera_stream',
              1,
              '#front-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)
                console.log('front camera ON msgs:', msgs)
                if (msgs[1].includes('Started camera_stream')) {
                  $('img#camera-feed')[0].src =
                    'http://' +
                    getRoverIP() +
                    ':8080/stream?topic=/cv_camera/image_raw'
                  $('img#camera-feed').addClass('rotateimg180')
                } else {
                  appendToConsole('Failed to open stream')
                  $('#front-camera-stream-btn')[0].checked = false
                }
              },
              '/dev/ttyFrontCam'
            )
          } else {
            requestTask(
              'camera_stream',
              0,
              '#front-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)
                console.log('front camera OFF msgs:', msgs)
                if (msgs[1].includes('Stopped camera_stream')) {
                  // succeeded to close stream
                  $('img#camera-feed')[0].src =
                    '../static/images/stream-offline.jpg'
                  $('img#camera-feed').removeClass('rotateimg180')
                } else {
                  // failed to close stream
                  // $('img#camera-feed')[0].src =
                  // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
                  appendToConsole('Failed to close stream')
                }
              },
              '/dev/ttyFrontCam'
            )
          }
        })

        $('#rear-camera-stream-btn').on('click', function (event) {
          event.preventDefault()
          // click makes it checked during this time, so trying to enable
          if ($('#rear-camera-stream-btn').is(':checked')) {
            requestTask(
              'camera_stream',
              1,
              '#rear-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)
                console.log('rear camera ON msgs:', msgs)
                if (msgs[1].includes('Started camera_stream')) {
                  $('img#camera-feed')[0].src =
                    'http://' +
                    getRoverIP() +
                    ':8080/stream?topic=/cv_camera/image_raw'
                } else {
                  appendToConsole('Failed to open stream')
                  $('#rear-camera-stream-btn')[0].checked = false
                }
              },
              '/dev/ttyRearCam'
            )
          } else {
            requestTask(
              'camera_stream',
              0,
              '#rear-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)
                if (msgs[1].includes('Stopped camera_stream')) {
                  console.log('rear camera OFF msgs:', msgs)

                  // succeeded to close stream
                  $('img#camera-feed')[0].src =
                    '../static/images/stream-offline.jpg'
                } else {
                  // failed to close stream
                  appendToConsole('Failed to close stream')
                  // $('img#camera-feed')[0].src =
                  // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
                }
              },
              '/dev/ttyRearCam'
            )
          }
        })

        $('#save-image').on('click', function (event) {
          $.ajax('/capture_image', {
            success: function (data) {
              appendToConsole(data.msg)
              if (!data.msg.includes('success')) {
                appendToConsole('Something went wrong, got', data.msg)
              } else {
                appendToConsole(data.msg)
              }
            },
            error: function () {
              console.log('An error occured')
            }
          })
        })

        $('#arm-science-camera-stream-btn').on('click', function (event) {
          event.preventDefault()
          // click makes it checked during this time, so trying to enable
          if ($('#arm-science-camera-stream-btn').is(':checked')) {
            requestTask(
              'camera_stream',
              1,
              '#arm-science-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)
                console.log('arm/science camera ON msgs:', msgs)
                if (msgs[1].includes('Started camera_stream')) {
                  $('img#camera-feed')[0].src =
                    'http://' +
                    getRoverIP() +
                    ':8080/stream?topic=/cv_camera/image_raw'
                } else {
                  appendToConsole('Failed to open stream')
                  $('#arm-science-camera-stream-btn')[0].checked = false
                }
              },
              '/dev/ttyArmScienceCam'
            )
          } else {
            requestTask(
              'camera_stream',
              0,
              '#arm-science-camera-stream-btn',
              function (msgs) {
                printErrToConsole(msgs)

                if (msgs[1].includes('Stopped camera_stream')) {
                  console.log('arm science camera OFF msgs:', msgs)

                  // succeeded to close stream
                  $('img#camera-feed')[0].src =
                    '../static/images/stream-offline.jpg'
                } else {
                  // failed to close stream
                  appendToConsole('Failed to close stream')
                  // $('img#camera-feed')[0].src =
                  // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
                }
              },
              '/dev/ttyArmScienceCam'
            )
          }
        })
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

  $('#flip-stream').on('click', function () {
    $('#camera-feed').toggleClass('rotateimg180')
  })

  $('#flip-stream-ccw').on('click', function () {
    $('#camera-feed').toggleClass('rotateimgccw')
    $('#camera-feed').toggleClass('stretch-down')
  })

  $('#flip-stream-cw').on('click', function () {
    $('#camera-feed').toggleClass('rotateimgcw')
    $('#camera-feed').toggleClass('stretch-down')
  })
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
      case 'Odroid':
      default:
        pingOdroid()
        break
    }
    lastCmdSent = new Date().getTime()
  }
}

/*
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
*/
