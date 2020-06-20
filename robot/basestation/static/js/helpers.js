// Console Log
const logConsole = '#write-to-log'

// Console Log
function appendToConsole (msg, devConsole = true, guiConsole = true) {
  if (guiConsole) {
    if($(logConsole)[0] != undefined)
    {
      $(logConsole).append(msg + '\n')
      $(logConsole).scrollTop($(logConsole)[0].scrollHeight)
    }
    else
    {
      console.log('Can\'t append message to log console. Logged messaged to chrome console')
      console.log(msg)
    }
  }
  if (devConsole) {
    console.log(msg + '\n')
  }
}

function printErrToConsole (msg) {
  if (!msg[0]) appendToConsole(msg[1])
}

// Rover log
// logs to console and scrolls to bottom
function appendToRoverLog (msg) {
  $('#rover-log').append(msg)
  $('#rover-log').scrollTop($('#rover-log')[0].scrollHeight)
}

function appendToScienceLog (msg) {
  $('#science-log').append(msg)
  $('#science-log').scrollTop($('#science-log')[0].scrollHeight)
}

function clearLogConsole () {
  $(logConsole).html('')
}

function scrollToBottom () {
  $(logConsole).scrollTop($(logConsole)[0].scrollHeight)
}

// Mocking data
function mockArmTableLog () {
  let m1 = 'm1: ' + $('#m1-angle').text()
  let m2 = 'm2: ' + $('#m2-angle').text()
  let m3 = 'm3: ' + $('#m3-angle').text()
  let m4 = 'm4: ' + $('#m4-angle').text()
  let m5 = 'm5: ' + $('#m5-angle').text()
  let m6 = 'm6: ' + $('#m6-angle').text()
  let motorAngleMsg = ['Motor angles:', m1, m2, m3, m4, m5, m6]

  appendToConsole(motorAngleMsg.join('\n') + '\n')
  scrollToBottom()
}

function mockRoverTableLog () {
  let m1 = 'Left-Front RPM:  ' + $('#left-front-rpm').text()
  let m2 = 'Left-Mid RPM:    ' + $('#left-mid-rpm').text()
  let m3 = 'Left-Rear RPM:   ' + $('#left-rear-rpm').text()
  let m4 = 'Right-Front RPM: ' + $('#right-front-rpm').text()
  let m5 = 'Right-Mid RPM:   ' + $('#right-mid-rpm').text()
  let m6 = 'Right-Rear RPM:  ' + $('#right-rear-rpm').text()
  let motorRpmMsg = ['Motor RPM:', m1, m2, m3, m4, m5, m6]

  appendToRoverLog(motorRpmMsg.join('\n') + '\n\n')
}

// Appends the passed bash and ros ping messages to the console log
function pingRover (ping_msg, ros_msg) {
  appendToConsole(ping_msg)
  appendToConsole(ros_msg)
  scrollToBottom()
}

// Updates the console log with odroid rx data
function updateOdroidRx () {
  $.ajax({
    url: '/odroid_rx',
    type: 'POST',
    success: function (response) {
      let newData = response.odroid_rx

      if (newData != $('#last-odroid-rx').val()) {
        appendToConsole('Odroid RX: ' + response.odroid_rx)
        scrollToBottom()
      }

      $('#last-odroid-rx').val(response.odroid_rx)
    }
  })
}

// Milliseconds since start time given
function millisSince (start) {
  var elapsed = new Date().getTime() - start
  return elapsed
}

// convenience function for UI Buttons
function color (selector, color) {
  $(selector).css('background-color', color)
}

function lightUp (selector) {
  color(selector, 'rgb(255, 0, 0)')
}

function dim (selector) {
  color(selector, 'rgb(74, 0, 0)')
}

function isLit (selector) {
  return (
    $(selector)
      .css('background-color')
      .toLowerCase() == 'rgb(255, 0, 0)'
  )
}

function greyOut (selector) {
  color(selector, '#6c757d')
}

// convenience function for setting text color of elements
function textColor (selector, color) {
  $(selector).css('color', color)
}

// convenience functions for setting/getting cookies
function setCookie (cname, cvalue, exdays) {
  var d = new Date()
  d.setTime(d.getTime() + exdays * 24 * 60 * 60 * 1000)
  var expires = 'expires=' + d.toUTCString()
  document.cookie = cname + '=' + cvalue + ';' + expires + ';path=/'
}

function getCookie (cname) {
  var name = cname + '='
  var decodedCookie = decodeURIComponent(document.cookie)
  var ca = decodedCookie.split(';')
  for (var i = 0; i < ca.length; i++) {
    var c = ca[i]
    while (c.charAt(0) == ' ') {
      c = c.substring(1)
    }
    if (c.indexOf(name) == 0) {
      return c.substring(name.length, c.length)
    }
  }
  return ''
}


// isNumeric
function isNumeric (num) {
  return (!isNaN(num) && !(num == '')) || num == 0
}

// AJAX
// Sends request to given route, prints the JSON response object
function sendRequest (msg) {
  var xhr = new XMLHttpRequest()

  xhr.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      // Typical action to be performed when the document is ready:
      console.log(JSON.parse(xhr.response))
    }
  }
  xhr.open('GET', msg, true)
  xhr.send(null)
}

function wait (ms) {
  var d = new Date()
  var d2 = null
  do {
    d2 = new Date()
  } while (d2 - d < ms)
}
