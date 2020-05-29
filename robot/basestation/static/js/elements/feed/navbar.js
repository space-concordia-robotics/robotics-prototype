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
  $("a[id^='mux-']").mouseup(e => {
    let muxId = $(e.target).attr("id")
    setMuxChannel(muxId)
  })

  // Set device channel
  function setMuxChannel (muxId) {
    deviceName = getDeviceNameByMuxID(muxId)
    if (isListenerOpen() && getCookie('serialType') == 'uart') {
      rosLog(ROSINFO, 'Don\'t change the mux channel while a listener is open!')
    } else {
      requestMuxChannel('#' + muxId, function (msgs) {
        printErrToConsole(msgs)
      })
    }
  }

  // set serialtype
  $('#uart').mouseup(setSerialUart)
  $('#usb').mouseup(setSerialUsb)

  function setSerialUart () {
    $('#serial-type').text('uart')
    setCookie('serialType', 'uart', 356)
    rosLog(ROSINFO, 'setting cookie to uart')
  }

  function setSerialUsb () {
    $('#serial-type').text('usb')
    setCookie('serialType', 'usb', 356)
    rosLog(ROSINFO, 'setting cookie to usb')
  }


  // send serial command based on mux channel and current page
  // beware that if choosing a different mux channel than the current page,
  // commands will probably mess something up until this is done in a smart manner
  $('#send-serial-btn').mouseup(function () {
    // b
    let command = $('#serial-command-input').val()
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
        // sendRoverCommand(command) // rover commands not yet implemented
      } else if (
        buttonText.includes('Arm') &&
        $('#toggle-arm-listener-btn')[0].checked == true
      ) {
        sendArmCommand(command)
      } else if (buttonText.includes('Science')) {
        // science buttons unknown
        // sendScienceCommand(command) // science commands not yet implemented
      } else if (buttonText.includes('PDS')) {
        // pds buttons unknown
        // sendPdsCommand(command) // pds commands not yet implemented
      }
      // no listener is open, send generic request
      else if (!buttonText.includes('Select Device Channel')) {
        requestSerialCommand(command, function (msgs) {
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

  $('#serial-command-input').on('keyup', function (e) {
    if (e.keyCode == 13) {
      // enter key
      // copy code from above
    }
  })

  $( '#arm-page' ).click(function() {
    window.open('/arm')
  })

  $( '#rover-page' ).click(function() {
    window.open('rover')
  })

  $( '#science-page' ).click(function() {
    window.open('science')
  })

  $( '#pds-page' ).click(function() {
    window.open('pds')
  })

  $( '#streams-page' ).click(function() {
    window.open('stream')
  })
})

/*
function to display the navbar modal with the given title and body text
*/
function navModalMessage (title, body){
  $('.modal-title').text(title)
  $('.modal-msg').text(body)
  $('.modal').modal({show: true})
}
