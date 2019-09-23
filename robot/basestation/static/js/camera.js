$(document).ready(() => {
  $('#front-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    console.log('fuck')
    // click makes it checked during this time, so trying to enable
    if ($('#front-camera-stream-btn').is(':checked')) {
      requestTask(
        'camera_stream',
        1,
        '#front-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)
          console.log('front camera ON msgs:', msgs)
          if (msgs[1].includes('Started camera_stream')) {
            $('img#camera-feed')[0].src =
              'http://' +
              getRoverIP() +
              ':8080/stream?topic=/cv_camera/image_raw'
            $('img#camera-feed').addClass('rotateimg180')
          } else {
            appendToConsole('Failed to open stream')
            $('#front-camera-stream-btn')[0].checked = false
          }
        },
        '/dev/ttyFrontCam'
      )
    } else {
      requestTask(
        'camera_stream',
        0,
        '#front-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)
          console.log('front camera OFF msgs:', msgs)
          if (msgs[1].includes('Stopped camera_stream')) {
            // succeeded to close stream
            $('img#camera-feed')[0].src =
              '../static/images/stream-offline.jpg'
            $('img#camera-feed').removeClass('rotateimg180')
          } else {
            // failed to close stream
            // $('img#camera-feed')[0].src =
            // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
            appendToConsole('Failed to close stream')
          }
        },
        '/dev/ttyFrontCam'
      )
    }
  })

  $('#rear-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#rear-camera-stream-btn').is(':checked')) {
      requestTask(
        'camera_stream',
        1,
        '#rear-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)
          console.log('rear camera ON msgs:', msgs)
          if (msgs[1].includes('Started camera_stream')) {
            $('img#camera-feed')[0].src =
              'http://' +
              getRoverIP() +
              ':8080/stream?topic=/cv_camera/image_raw'
          } else {
            appendToConsole('Failed to open stream')
            $('#rear-camera-stream-btn')[0].checked = false
          }
        },
        '/dev/ttyRearCam'
      )
    } else {
      requestTask(
        'camera_stream',
        0,
        '#rear-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)
          if (msgs[1].includes('Stopped camera_stream')) {
            console.log('rear camera OFF msgs:', msgs)

            // succeeded to close stream
            $('img#camera-feed')[0].src =
              '../static/images/stream-offline.jpg'
          } else {
            // failed to close stream
            appendToConsole('Failed to close stream')
            // $('img#camera-feed')[0].src =
            // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
          }
        },
        '/dev/ttyRearCam'
      )
    }
  })

  $('#save-image').on('click', function (event) {
    $.ajax('/capture_image', {
      success: function (data) {
        appendToConsole(data.msg)
        if (!data.msg.includes('success')) {
          appendToConsole('Something went wrong, got', data.msg)
        } else {
          appendToConsole(data.msg)
        }
      },
      error: function () {
        console.log('An error occured')
      }
    })
  })

  $('#arm-science-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#arm-science-camera-stream-btn').is(':checked')) {
      requestTask(
        'camera_stream',
        1,
        '#arm-science-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)
          console.log('arm/science camera ON msgs:', msgs)
          if (msgs[1].includes('Started camera_stream')) {
            $('img#camera-feed')[0].src =
              'http://' +
              getRoverIP() +
              ':8080/stream?topic=/cv_camera/image_raw'
          } else {
            appendToConsole('Failed to open stream')
            $('#arm-science-camera-stream-btn')[0].checked = false
          }
        },
        '/dev/ttyArmScienceCam'
      )
    } else {
      requestTask(
        'camera_stream',
        0,
        '#arm-science-camera-stream-btn',
        function (msgs) {
          printErrToConsole(msgs)

          if (msgs[1].includes('Stopped camera_stream')) {
            console.log('arm science camera OFF msgs:', msgs)

            // succeeded to close stream
            $('img#camera-feed')[0].src =
              '../static/images/stream-offline.jpg'
          } else {
            // failed to close stream
            appendToConsole('Failed to close stream')
            // $('img#camera-feed')[0].src =
            // 'http://' + getRoverIP() + ':8080/stream?topic=/cv_camera/image_raw'
          }
        },
        '/dev/ttyArmScienceCam'
      )
    }
  })
})
