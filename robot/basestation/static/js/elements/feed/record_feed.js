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
          if (jqXHR.status === 0) {
            recording_log_msg = 'cannot connect verify network'
          } else if (jqXHR.status == 404) {
            recording_log_msg = 'requested page not found [404]'
          } else if (jqXHR.status == 500) {
            recording_log_msg = 'internal Server Error [500]'
          } else if (exception === 'parsererror') {
            recording_log_msg = 'requested JSON parse failed'
          } else if (exception === 'timeout') {
            recording_log_msg = 'time out error'
          } else if (exception === 'abort') {
            recording_log_msg = 'Ajax request aborted'
          } else {
            recording_log_msg = 'Uncaught Error. ' + jqXHR.responseText
          }
          appendToConsole(recording_log_msg)
        }
      })
    } else {
      $.ajax('/stop_feed_recording/' + stream, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          toggleText(recordingButton)
        }
      })
    }
  }

  function toggleText (button) {
    if (button.text() == button.data('text-swap')) {
      button.text(button.data('text-original'))
    } else {
      button.data('text-original', button.text())
      button.text(button.data('text-swap'))
    }
  }
})
