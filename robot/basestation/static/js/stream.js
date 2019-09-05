<<<<<<< Updated upstream
$(document).ready(() => {
  $('#arm-stream-btn').on('click', function (event) {
      event.preventDefault()
      // click makes it checked during this time, so trying to enable
      if ($('#arm-stream-btn').is(':checked')) {
        requestTask(
          'camera_stream',
          1,
          '#arm-camera-stream-btn',
          function (msgs) {
            printErrToConsole(msgs)
            console.log('arm/science camera ON msgs:', msgs)
            if (msgs[1].includes('Started camera_stream')) {
              $('img#camera-feed')[0].src =
                'http://' +
                getRoverIP() +
                ':8080/stream?topic=/ArmCamera/image_raw'
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
          '#arm-camera-stream-btn',
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

  $('#front-camera-stream-btn').on('click', function (event) {
      event.preventDefault()
      // click makes it checked during this time, so trying to enable
      if ($('#front-camera-stream-btn').is(':checked')) {
        requestTask(
          'camera_stream',
          1,
          '#front-stream-btn',
          function (msgs) {
            printErrToConsole(msgs)
            console.log('front camera ON msgs:', msgs)
            if (msgs[1].includes('Started camera_stream')) {
              $('img#front-camera-feed')[0].src =
                'http://' +
                getRoverIP() +
                ':8080/stream?topic=/FrontCamera/image_raw'
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
          '#front-stream-btn',
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

  $('#rear-stream-btn').on('click', function (event) {
      event.preventDefault()
      // click makes it checked during this time, so trying to enable
      if ($('#rear-stream-btn').is(':checked')) {
          $('img#rear-camera-feed')[0].src =
          'localhost:8080/stream_viewer?topic=/RearCamera/image_raw'
      } else {
          $('img#arm-camera-feed')[0].src =
              '../static/images/stream-offline.jpg'
      }
  })
=======
$(document).ready(function () {
    $('#arm-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#arm-stream-btn').is(':checked')) {
            $('img#arm-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/ArmCamera/image_raw'
        } else {
            $('img#arm-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })
    $('#front-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#front-stream-btn').is(':checked')) {
            $('img#front-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/FrontCamera/image_raw'
        } else {
            $('img#front-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })

    $('#rear-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#rear-stream-btn').is(':checked')) {
            $('img#rear-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/RearCamera/image_raw'
        } else {
            $('img#rear-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })
>>>>>>> Stashed changes
})
