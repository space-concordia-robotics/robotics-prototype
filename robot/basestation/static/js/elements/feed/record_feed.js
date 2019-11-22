$(document).ready(() => {
  window.onload = function () {
    keepText($('#record-feed-toggle'))
    $('#record-feed-toggle').css({ 'font-weight': 'bold' })
    // Add more calls to keeptext when adding additional recording buttons
  }

  $('#record-feed-toggle').click(function () {
    const recordingButton = $(this)
    const stream = 'default'
    toggleRecording(recordingButton, stream)
    recordingButton.css({ 'font-weight': 'bold' })
  })

  function toggleRecording (recordingButton, stream) {
    if (recordingButton.text() != recordingButton.data('text-swap')) {
      const ajax_url = '/initiate_feed_recording/' + stream
      $.ajax(ajax_url, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          if (data.error_state == 0) {
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
          appendToConsole(data.recording_log_msg)
          toggleText(recordingButton)
        },

        error: function (jqXHR, exception) {
          flaskError(jqXHR, exception, ajax_url)
        }
      })
    }
  }
})
