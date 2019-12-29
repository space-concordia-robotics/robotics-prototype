$(document).ready(() => {
  const RECORDING_ON = "../../../static/img/camera/record_on.png";
  const RECORDING_OFF = "../../../static/img/camera/record_off.png";
        
  // Call on keepText function with recording buttons on page load
  window.onload = function () {
    //keepText($('#record-feed-toggle'))
    //$('#record-feed-toggle').css({ 'font-weight': 'bold' })
    // Add more calls to keeptext when adding additional recording buttons
  }

  function toggleRecordingButton(recordingButton, isRecording)
  {
    let recordingAttr = isRecording ? "true" : "false";
    let sourceAttr = isRecording ? RECORDING_ON : RECORDING_OFF;

    recordingButton.attr("recording", recordingAttr);
    recordingButton.attr("src", sourceAttr);
  }

  // Button listener to run recording toggle with stream identifier
  $('.camera-recording').click(function () {
    const recordingButton = $(this);
    const stream = 'default'
    toggleRecording(recordingButton, stream)
  })

  // Toggle python recording functions depending on current button state and handle some errors
  function toggleRecording (recordingButton, stream) {
    let isRecording = recordingButton.attr("recording") == 'true';
    if (!isRecording) {
      toggleRecordingButton(recordingButton, true);
      const ajax_url = '/initiate_feed_recording/' + stream
      $.ajax(ajax_url, {
        success: function (data) {
          appendToConsole(data.recording_log_msg)
          if (data.error_state == 1) {
            toggleRecordingButton(recordingButton, false);
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
            toggleRecordingButton(recordingButton, false);
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
