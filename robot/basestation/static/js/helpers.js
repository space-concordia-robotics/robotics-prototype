// Console Log
const logConsole = '#write-to-log'
const serialCmd = '#serial-cmd-input'

// Console Log
function appendToConsole (msg) {
  $(logConsole).append(msg + '\n')
}

// Rover log
// logs to console and scrolls to bottom
function appendToRoverLog (msg) {
  $('#rover-log').append(msg)
  $('#rover-log').scrollTop($('#rover-log')[0].scrollHeight)
}

function clearLogConsole () {
  $(logConsole).html('')
}

function clearSerialCmd () {
  $(serialCmd).val('')
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
// Manual control
function manualControl () {
  var a = document.getElementById('ArmcontrolsOFF')
  var b = document.getElementById('ArmcontrolsON')

  if (a.style.display === 'none') {
    a.style.display = 'block'
    b.style.display = 'none'
  } else {
    a.style.display = 'none'
    b.style.display = 'block'
    b.style.borderRadius = '0'
  }
}

function toggleToManual () {
  if (!$('#manual-control-btn')[0].checked) {
    $('#manual-control-btn').click()
  }
}


// Milliseconds since start time given
function millisSince (start) {
  var elapsed = new Date().getTime() - start
  return elapsed
}

// convenience function for UI Buttons
function lightUp(selector) {
    $(selector).css('background-color', 'rgb(255, 0, 0)')
}

function dim(selector) {
    $(selector).css('background-color', 'rgb(74, 0, 0)')
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
