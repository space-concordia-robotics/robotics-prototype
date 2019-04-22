// @TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

// feature toggle for mock data in rover table
let mockRoverTable = false

// constants for speed setting limits (absolute max: 45)
const MAX_THROTTLE_SPEED = 25
const MIN_THROTTLE_SPEED = 0
const MAX_STEERING_SPEED = 39
const MIN_STEERING_SPEED = 0

// for command thoughput limiting
const DRIVE_THROTTLE_TIME = 25
const PING_THROTTLE_TIME = 1000
const MCU_FEEDBACK_THROTTLE = 1000
const LISTENER_TOGGLE_THROTTLE = 3000
var lastCmdSent = 0

function enableRoverListener () {
  if ($('#enable-rover-btn').is(':checked')) {
    $.ajax({
      url: '/task_handler',
      type: 'POST',
      data: {
        cmd: 'enable-rover-listener'
      },
      success: function (response) {
        appendToConsole('cmd: ' + response.cmd)
        appendToConsole('output: ' + response.output)
        if (response.error != 'None') {
          appendToConsole('error:\n' + response.error)
        }
        scrollToBottom()
      }
    })
  } else {
    $.ajax({
      url: '/task_handler',
      type: 'POST',
      data: {
        cmd: 'disable-rover-listener'
      },
      success: function (response) {
        appendToConsole('cmd: ' + response.cmd)
        appendToConsole('output: ' + response.output)
        if (response.error != 'None') {
          appendToConsole('error:\n' + response.error)
        }
        scrollToBottom()
      }
    })
  }
}

// for updating the toggle buttons if user pressed keyboard events triggered enabling/disabling
function enableRoverMotorsBtn () {
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).removeClass(
    'btn-danger off'
  )
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).addClass(
    'btn-success'
  )
  $('#enable-rover-motors-btn')[0].checked = true
}

function disableRoverMotorsBtn () {
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).addClass(
    'btn-danger off'
  )
  $($('.toggle > #enable-rover-motors-btn')[0].parentNode).removeClass(
    'btn-success'
  )
  $('#enable-rover-motors-btn')[0].checked = false
}

function enableRoverListenerBtn () {
  $($('.toggle > #toggle-rover-listener')[0].parentNode).removeClass(
    'btn-danger off'
  )
  $($('.toggle > #toggle-rover-listener')[0].parentNode).addClass('btn-success')
  $('#toggle-rover-listener')[0].checked = true
}

function disableRoverListenerBtn () {
  $($('.toggle > #toggle-rover-listener')[0].parentNode).addClass(
    'btn-danger off'
  )
  $($('.toggle > #toggle-rover-listener')[0].parentNode).removeClass(
    'btn-success'
  )
  $('#toggle-rover-listener')[0].checked = false
}

// commands to change speed settings, get buffered serial messages
$(document).keydown(function (e) {
  let currentSpeed = ''

  switch (e.which) {
    case 73: // 'i' --> increase throttle
      $('#throttle-increase > button').css('background-color', 'rgb(255, 0, 0)')
      currentSpeed = $('#throttle-speed').text()

      if (currentSpeed < MAX_THROTTLE_SPEED) {
        $('#throttle-speed').text(parseFloat(currentSpeed) + 0.5)
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'i'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break

    case 74: // 'j' --> decrease throttle
      $('#throttle-decrease > button').css('background-color', 'rgb(255, 0, 0)')
      currentSpeed = $('#throttle-speed').text()

      if (currentSpeed > MIN_THROTTLE_SPEED) {
        $('#throttle-speed').text(parseFloat(currentSpeed) - 0.5)
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'j'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break

    case 79: // 'o' --> increase steering
      $('#steering-increase > button').css('background-color', 'rgb(255, 0, 0)')
      currentSpeed = $('#steering-speed').text()

      if (currentSpeed < MAX_STEERING_SPEED) {
        $('#steering-speed').text(parseFloat(currentSpeed) + 0.5)
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'o'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break

    case 75: // 'k' --> decrease steering
      $('#steering-decrease > button').css('background-color', 'rgb(255, 0, 0)')
      currentSpeed = $('#steering-speed').text()

      if (currentSpeed > MIN_STEERING_SPEED) {
        $('#steering-speed').text(parseFloat(currentSpeed) - 0.5)
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'k'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break

    case 66: // 'b' --> get buffered serial messages
      if (millisSince(lastCmdSent) > MCU_FEEDBACK_THROTTLE) {
        $('button#show-buffered-rover-msgs').css(
          'background-color',
          'rgb(255, 0, 0)'
        )
        $.ajax({
          url: '/rover_drive',
          type: 'POST',
          data: {
            cmd: 'b'
          },
          success: function (response) {
            appendToConsole('cmd: ' + response.cmd)
            appendToConsole('feedback:\n' + response.feedback)
            if (response.error != 'None') {
              appendToConsole('error:\n' + response.error)
            }
            scrollToBottom()
          }
        })
        lastCmdSent = new Date().getTime()
      }
      break
    case 77: // 'm' --> enable motor control
      $('button#enable-rover-motors').css('background-color', 'rgb(255, 0, 0)')
      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'm'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            enableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 78: // 'n' --> disable motor control
      $('button#disable-rover-motors').css('background-color', 'rgb(255, 0, 0)')
      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'n'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            disableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 76: // 'l' --> list all commands
      $('button#list-all-rover-cmds').css('background-color', 'rgb(255, 0, 0)')
      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'l'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (!response.feedback.includes('limit exceeded')) {
            disableRoverMotorsBtn()
          }
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
      break
    case 84: // 't' --> toggle listener proxy script
      if (millisSince(lastCmdSent) > LISTENER_TOGGLE_THROTTLE) {
        $('button#toggle-rover-listener-btn').css(
          'background-color',
          'rgb(255, 0, 0)'
        )

        let request = ''

        if ($('#toggle-rover-listener').is(':checked')) {
          request = 'disable-rover-listener'
        } else {
          request = 'enable-rover-listener'
        }

        $.ajax({
          url: '/task_handler',
          type: 'POST',
          data: {
            cmd: request
          },
          success: function (response) {
            console.log(response)
            appendToConsole('cmd: ' + response.cmd)
            appendToConsole('output: ' + response.output)
            if (
              !response.output.includes('Failed') &&
              !response.output.includes('shutdown request')
            ) {
              if ($('#toggle-rover-listener').is(':checked')) {
                disableRoverListenerBtn()
              } else {
                enableRoverListenerBtn()
              }
            }

            if (response.error != 'None') {
              appendToConsole('error:\n' + response.error)
            }
            scrollToBottom()
          }
        })
        lastCmdSent = new Date().getTime()
      }
      break
    case 81: // 'q' --> terminate (quit) listener script
        $('button#terminate-listener-script').css(
          'background-color',
          'rgb(255, 0, 0)'
        )
        $.ajax({
          url: '/rover_drive',
          type: 'POST',
          data: {
            cmd: 'q'
          },
          success: function (response) {
            appendToConsole('cmd: ' + response.cmd)
            appendToConsole('feedback:\n' + response.feedback)
            if (!response.feedback.includes('limit exceeded')) {
              disableRoverListenerBtn()
            }
            if (response.error != 'None') {
              appendToConsole('error:\n' + response.error)
            }
            scrollToBottom()
          }
        })
        lastCmdSent = new Date().getTime()
      break
    default:
      return // exit this handler for other keys
  }
  e.preventDefault() // prevent the default action (scroll / move caret)
})

// no throttling necessary as since keydown events are throttled
// those keys will not change color and the following code will only set it to it's default color
$(document).keyup(function (e) {
  switch (e.which) {
    case 65: // left
      $('#rover-left > button').css('background-color', 'rgb(74, 0, 0)')

      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      break

    case 87: // up
      $('#rover-up > button').css('background-color', 'rgb(74, 0, 0)')
      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      break

    case 68: // right
      $('#rover-right > button').css('background-color', 'rgb(74, 0, 0)')
      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      break

    case 83: // down
      $('#rover-down > button').css('background-color', 'rgb(74, 0, 0)')
      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      break

    case 73: // increase throttle
      $('#throttle-increase > button').css('background-color', 'rgb(74, 0, 0)')
      break

    case 74: // decrease throttle
      $('#throttle-decrease > button').css('background-color', 'rgb(74, 0, 0)')
      break

    case 79: // increase steering
      $('#steering-increase > button').css('background-color', 'rgb(74, 0, 0)')
      break

    case 75: // decrease steering
      $('#steering-decrease > button').css('background-color', 'rgb(74, 0, 0)')
      break

    case 77: // enable rover motors
      $('button#enable-rover-motors').css('background-color', 'rgb(74, 0, 0)')
      break

    case 78: // disable rover motors
      $('button#disable-rover-motors').css('background-color', 'rgb(74, 0, 0)')
      break
    case 66: // show buffered serial messages from the MCU
      $('button#show-buffered-rover-msgs').css(
        'background-color',
        'rgb(74, 0, 0)'
      )
      break
    case 76: // list all rover cmds
      $('button#list-all-rover-cmds').css('background-color', 'rgb(74, 0, 0)')
      break
    case 84: // toggle rover listener proxy script
      $('button#toggle-rover-listener-btn').css(
        'background-color',
        'rgb(74, 0, 0)'
      )
      break
    case 81: // terminate listener script (quit)
      $('button#terminate-listener-script').css(
        'background-color',
        'rgb(74, 0, 0)'
      )
      break

    default:
      return // exit this handler for other keys
  }
  e.preventDefault() // prevent the default action (scroll / move caret)
})

// GAME LOOP CONTROL

var keyState = {}
window.addEventListener(
  'keydown',
  function (e) {
    keyState[e.keyCode || e.which] = true
  },
  true
)
window.addEventListener(
  'keyup',
  function (e) {
    keyState[e.keyCode || e.which] = false
  },
  true
)

function gameLoop () {
  if (millisSince(lastCmdSent) > DRIVE_THROTTLE_TIME) {
    // 'a' --> rover turn left
    if (keyState[65]) {
      $('#rover-left > button').css('background-color', 'rgb(255, 0, 0)')

      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'a'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })

      lastCmdSent = new Date().getTime()
    }
    // 'w' --> rover forward
    else if (keyState[87]) {
      $('#rover-up > button').css('background-color', 'rgb(255, 0, 0)')

      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'w'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
    }
    // 'd' --> rover right
    else if (keyState[68]) {
      $('#rover-right > button').css('background-color', 'rgb(255, 0, 0)')

      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 'd'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
    }

    // 's' --> rover back
    else if (keyState[83]) {
      $('#rover-down > button').css('background-color', 'rgb(255, 0, 0)')

      if (mockRoverTable) {
        $('#left-front-rpm').text('0')
        $('#left-front-current').text('0.3')
        $('#left-mid-rpm').text('0')
        $('#left-mid-current').text('0.3')
        $('#left-rear-rpm').text('0')
        $('#left-rear-current').text('0.3')
        $('#right-front-rpm').text('0')
        $('#right-front-current').text('0.3')
        $('#right-mid-rpm').text('0')
        $('#right-mid-current').text('0.3')
        $('#right-rear-rpm').text('0')
        $('#right-rear-current').text('0.3')
      }

      $.ajax({
        url: '/rover_drive',
        type: 'POST',
        data: {
          cmd: 's'
        },
        success: function (response) {
          appendToConsole('cmd: ' + response.cmd)
          appendToConsole('feedback:\n' + response.feedback)
          if (response.error != 'None') {
            appendToConsole('error:\n' + response.error)
          }
          scrollToBottom()
        }
      })
      lastCmdSent = new Date().getTime()
    }
    // send REST signal, mimicking behavior of bluetooth setup
    // else {
    // $.ajax({
    //     url: '/rover_drive',
    //     type: 'POST',
    //     data: {
    //         cmd: 'k'
    //     },
    //     success: function(response){
    //         appendToConsole("cmd: " + response.cmd);
    //         scrollToBottom();
    //     }
    // })
    // }
    // redraw/reposition your object here
    // also redraw/animate any objects not controlled by the user
  }

  setTimeout(gameLoop, 10)
}

gameLoop()

if (mockRoverTable) {
  setInterval(mockRoverTableLog, 1000)
}
