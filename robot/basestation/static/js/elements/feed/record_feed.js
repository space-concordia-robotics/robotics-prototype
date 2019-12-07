$(document).ready(() => {
  // Call on keepText function with recording buttons on page load
  window.onload = function () {
    keepText($('#record-feed-toggle'))
    $('#record-feed-toggle').css({ 'font-weight': 'bold' })
    // Add more calls to keeptext when adding additional recording buttons
  }

  // Button listener to run recording toggle with stream identifier
  $('#record-feed-toggle').click(function () {
    const recordingButton = $(this)
    const stream = 'default'
    toggleRecording(recordingButton, stream)
    recordingButton.css({ 'font-weight': 'bold' })
  })

  // Toggle python recording functions depending on current button text and handle some errors
  function toggleRecording (recordingButton, stream) {
    if (recordingButton.text() != recordingButton.data('text-swap')) {
      toggleText(recordingButton)
      const ajax_url = '/initiate_feed_recording/' + stream
      $.ajax(ajax_url, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          if (data.error_state == 1) {
            toggleText(recordingButton)
          }
        },

        error: function (jqXHR, exception) {
          flaskError(jqXHR, exception, ajax_url)
        }
      })
    } else {
      const ajax_url = '/stop_feed_recording/' + stream
      $.ajax(ajax_url, {
        success: function (data) {
          if (data.error_state == 0) {
            toggleText(recordingButton)
            appendToConsole(data.recording_log_msg)
          }
        },

        error: function (jqXHR, exception) {
          flaskError(jqXHR, exception, ajax_url)
        }
      })
    }
  }
})
