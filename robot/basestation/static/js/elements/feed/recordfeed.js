$(document).ready(() => {
  const RECORDING_ON = "../../../static/img/camera/record_on.png";
  const RECORDING_OFF = "../../../static/img/camera/record_off.png";


  // Toggles the recording button's image
  function toggleRecordingButton(recordingButton, isRecording)
  {
    let recordingAttr = isRecording ? "true" : "false";
    let sourceAttr = isRecording ? RECORDING_ON : RECORDING_OFF;

    recordingButton.attr("recording", recordingAttr);
    recordingButton.attr("src", sourceAttr);
    window.localStorage.setItem(recordingButton, recordingAttr)
  }

  $('.camera-recording').click(function () {
    const recordingButton = $(this);
    const stream = 'default'
    toggleRecording(recordingButton, stream)
  })

  function startRecording(stream, callback = () => {}) {
    const start_recording_url = '/initiate_feed_recording/' + stream
    $.ajax(start_recording_url, {
      success: data => {
        appendToConsole(data.recording_log_msg);
        callback(data.error_state != 1);
      }, 
      error: (jqXHR, exception) => {
        flaskError(jqXHR, exception, start_recording_url);
      }
    });
  }

  function stopRecording(stream, callback = () => {}) {
    const stop_recording_url = '/stop_feed_recording/' + stream
    $.ajax(stop_recording_url, {
      success: data => {
        appendToConsole(data.recording_log_msg);
        callback(data.error_state != 1);
      }, 
      error: (jqXHR, exception) => {
        flaskError(jqXHR, exception, start_recording_url);
      }
    });
  }

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
