$(document).ready(() => {
  $('#record-feed-toggle').click(function () {
    const recordingButton = $(this)
    const stream = 'default'
    toggleRecording(recordingButton, stream)
    recordingButton.css({ 'font-weight': 'bold' })
  })

  function toggleRecording (recordingButton, stream) {
    if (recordingButton.text() != recordingButton.data('text-swap')) {
      $.ajax('/initiate_feed_recording/' + stream, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          if (data.error_state == 0) {
            toggleText(recordingButton)
          }
        },

        error: function (jqXHR, exception) {
          flaskError(jqXHR, exception)
        }
      })
    } else {
      $.ajax('/stop_feed_recording/' + stream, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          toggleText(recordingButton)
        },

        error: function (jqXHR, exception) {
          flaskError(jqXHR, exception)
        }
      })
    }
  }
})
