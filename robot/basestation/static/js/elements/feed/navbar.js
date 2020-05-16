$(document).ready(() => {

  function append_css(file) {
    console.log('Append CSS: ' + file)

      var link = document.createElement('link')
      link.href = '/static/css/' + file + '.css'
      link.type = 'text/css'
      link.rel = 'stylesheet'
      link.media = 'screen,print'

      document.getElementsByTagName('head')[0].appendChild(link)
  }

  function prefer_theme(name) {
    console.log('Prefer theme: ' + name)
      setCookie('theme', name, 365)  // setCookie() from helpers.js
      append_css('themes/' + name)
  }

  preferred_theme = getCookie('theme')  // getCookie() from helpers.js
    if (preferred_theme != '') {
      prefer_theme(preferred_theme)
    } else {
      prefer_theme('lofi')
    }

  $('#theme-mantis').click(function () {
    console.log('theme-mantis')
      prefer_theme('mantis')
  })
  $('#theme-lofi').click(function () {
    console.log('theme-lofi')
      prefer_theme('lofi')
  })


  function checkListenerStates() {
    let listener = ''
      let url = window.location.pathname
      let toggleButtonID = 'toggle-'

      if (url == '/rover') {
        listener = ROVER_LISTENER_TASK
          toggleButtonID += 'rover-listener-btn'
      } else if (url == '/' || url == '/arm') {
        listener = ARM_LISTENER_TASK
          toggleButtonID += 'arm-listener-btn'
      } else if (url == '/science') {
        listener = SCIENCE_LISTENER_TASK
          toggleButtonID += 'science-listener-btn'
      } else if (url == '/pds') {
        listener = PDS_LISTENER_TASK
          toggleButtonID += 'pds-listener-btn'
      }
      else{
        return; // Pages that don't have this button should not run the code below (See #367)
      }

    let serialType = $('#serial-type').text()

      requestTask(
          listener,
          STATUS_CHECK,
          function (msgs) {
            console.log('msgs', msgs)
              if (msgs[1].includes('not') || msgs[1].includes('timeout')) {
                $('#' + toggleButtonID)[0].checked = false
              } else {
                $('#' + toggleButtonID)[0].checked = true
              }
          },
          serialType
          )
  }

  checkListenerStates()

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
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      appendToConsole('Don\'t change the mux channel while a listener is open!')
    } else {
      requestMuxChannel('#mux-0', function (msgs) {
        printErrToConsole(msgs)

          if (msgs[0] == true && window.location.pathname == '/rover') {
            console.log('Activating Rover Listener Node')

              let serialType = getCookie('serialType')

              if (serialType == '') {
                appendToConsole('Serial type not yet defined!')
                  return
              }

            // automating opening listener and sending MCU ping in UART mode
            if (serialType == 'uart') {
              requestTask(
                  ROVER_LISTENER_TASK,
                  STATUS_START,
                  function (msgs) {
                    if (msgs[0]) {
                      $('#toggle-rover-listener-btn')[0].checked = true
                        // try pinging MCU
                        wait(1000)
                        sendRequest('Rover', 'ping', printErrToConsole)
                    } else {
                      $('#toggle-rover-listener-btn')[0].checked = false
                    }
                  },
                  serialType
                  )
            }
          }
      })
    }
  })

  $('#mux-1').mouseup(function () {
    // Arm
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      appendToConsole('Don\'t change the mux channel while a listener is open!')
    } else {
      requestMuxChannel('#mux-1', function (msgs) {
        printErrToConsole(msgs)

          if (msgs[0] == true && window.location.pathname == '/') {
            console.log('Activating Arm Listener Node')

              let serialType = getCookie('serialType')

              if (serialType == '') {
                appendToConsole('Serial type not yet defined!')
                  return
              }

            // automating opening listener and sending MCU ping in UART mode
            if (serialType == 'uart') {
              requestTask(
                  ARM_LISTENER_TASK,
                  STATUS_START,
                  function (msgs) {
                    if (msgs[0]) {
                      $('#toggle-arm-listener-btn')[0].checked = true
                        // try pinging MCU
                        wait(1000)
                        sendRequest('Arm', 'ping', printErrToConsole)
                    } else {
                      $('#toggle-arm-listener-btn')[0].checked = false
                    }
                  },
                  serialType
                  )
            }
          }
      })
    }
  })

  $('#mux-2').mouseup(function () {
    // Science
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      appendToConsole('Don\'t change the mux channel while a listener is open!')
    } else {
      requestMuxChannel('#mux-2', function (msgs) {
        printErrToConsole(msgs)

          if (msgs[0] == true && window.location.pathname == '/science') {
            console.log('Activating Science Listener Node')

              let serialType = getCookie('serialType')

              if (serialType == '') {
                appendToConsole('Serial type not yet defined!')
                  return
              }

            // automating opening listener and sending MCU ping in UART mode
            if (serialType == 'uart') {
              requestTask(
                  SCIENCE_LISTENER_TASK,
                  STATUS_START,
                  function (msgs) {
                    if (msgs[0]) {
                      $('#science-listener-btn')[0].checked = true
                        // try pinging MCU
                        wait(1000)
                        sendRequest('Science', 'ping', printErrToConsole)
                    } else {
                      $('#science-listener-btn')[0].checked = false
                    }
                  },
                  serialType
                  )
            }
          }
      })
    }
  })

  $('#mux-3').mouseup(function () {
    // PDS
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      appendToConsole('Don\'t change the mux channel while a listener is open!')
    } else {
      requestMuxChannel('#mux-3', function (msgs) {
        printErrToConsole(msgs)
      })
    }
  })

  // select mux channel using mux_select service
  $('#mux-0').mouseup(function() {
    // Rover
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      appendToConsole("Don't change the mux channel while a listener is open!")
    } else {
      requestMuxChannel('#mux-0', function(msgs) {
        printErrToConsole(msgs)

          if (msgs[0] == true && window.location.pathname == '/rover') {
            console.log('Activating Rover Listener Node')

              let serialType = getCookie('serialType')

              if (serialType == '') {
                appendToConsole('Serial type not yet defined!')
                  return
              }

            // automating opening listener and sending MCU ping in UART mode
            if (serialType == 'uart') {
              requestTask(
                  'rover_listener',
                  1,
                  '#toggle-rover-listener-btn',
                  function(msgs) {
                    if (msgs[0]) {
                      $('#toggle-rover-listener-btn')[0].checked = true
                        // try pinging MCU
                        wait(1000)
                        sendRequest('Rover', 'ping', printErrToConsole)
                    } else {
                      $('#toggle-rover-listener-btn')[0].checked = false
                    }
                  },
                  serialType
                  )
            }
          }
      })
    }
  })

  $('#uart').mouseup(function() {
    $('#serial-type').text('uart')
      setCookie('serialType', 'uart', 3)
      appendToConsole('setting cookie to uart')
  })

  $('#usb').mouseup(function() {
    $('#serial-type').text('usb')
      setCookie('serialType', 'usb', 3)
      appendToConsole('setting cookie to usb')
  })

  // send serial command based on mux channel and current page
  // beware that if choosing a different mux channel than the current page,
  // commands will probably mess something up until this is done in a smart manner
  $('#send-serial-btn').mouseup(function() {
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
          requestSerialCommand(cmd, function(msgs) {
            console.log(msgs)
              if (msgs[0]) {
                console.log('nice')
              } else { console.log('not nice') } })
        }
      }
  })

  $('#serial-cmd-input').on('keyup', function(e) {
    if (e.keyCode == 13) {
      // enter key
      // copy code from above
    }
  })

  $("#arm-page").click(function() {
    window.open('arm');
  })

  $("#rover-page").click(function() {
    window.open('rover');
  })

  $("#science-page").click(function() {
    window.open('science');
  })

  $("#pds-page").click(function() {
    window.open('pds');
  })

  $("#streams-page").click(function() {
    window.open('stream');
  })
  $("#navigation-page").click(function() {
    window.open('navigation');
  })
})

/*
  Display the navbar modal with the given title and body text
*/
function navModalMessage (title, body){
  $('.modal-title').text(title)
    $('.modal-msg').text(body)
    $('.modal').modal({show: true})
}
