$(document).ready(() => {
  $('#front-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#front-camera-stream-btn').is(':checked')) {
      requestTask(
        'camera_stream',
        STATUS_START,
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
        STATUS_STOP,
        function (msgs) {
          printErrToConsole(msgs)
          console.log('front camera OFF msgs:', msgs)
          if (msgs[1].includes('Stopped camera_stream')) {
            // succeeded to close stream
            $('img#camera-feed')[0].src = '../static/img/stream-offline.jpg'
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

  $('.camera-screenshot').on('click', function (event) {
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

  // Rotate camera feed 90 degrees CCW
  $('.camera-rotl').click(e => {
    let cameraPanel = $(e.target).parents(".camera-panel");
    let rotation = cameraPanel.attr("rotation");
    rotation = negativeModulo(--rotation, 4);
    cameraPanel.attr("rotation", rotation);
    let angle = rotation * 90;
    let cameraFeed = cameraPanel.find(".camera-feed");
    rotateElement(cameraFeed, angle)
  })

  // Rotate camera feed 90 degrees CW
  $('.camera-rotr').click(e => {
    let cameraPanel = $(e.target).parents(".camera-panel");
    let rotation = cameraPanel.attr("rotation");
    rotation = negativeModulo(++rotation, 4);
    cameraPanel.attr("rotation", rotation);
    let angle = rotation * 90;
    let cameraFeed = cameraPanel.find(".camera-feed");
    rotateElement(cameraFeed, angle)
  })

  function setStreamSelection (availableStreams) {
    availableStreams.forEach(elt => {
      $('.camera-selections').append(
        '<li class="camera-selection-element">' + elt + '</li>'
      )
    })
  }

  $(document).on('click', '.camera-selection-element', e => {
    let selectedStream = $(e.target)[0]
    let cameraPanel = $(e.target).parents(".camera-panel");
    let cameraName = cameraPanel.find(".camera-name");
    cameraName[0].innerHTML = selectedStream.innerHTML
  })

  $(".camera-stream").hover(inEvent => {
    let cameraStream = $(inEvent.currentTarget);
    let cameraControls = cameraStream.children(".camera-controls");
    cameraControls.css("display", "block");

  }, outEvent => {
    let cameraStream = $(outEvent.currentTarget);
    let cameraControls = cameraStream.children(".camera-controls");
    cameraControls.css("display", "none");
  });

  addIdentifiers(".camera-panel");
  setStreamSelection(['/dev/videoFrontCam', '/dev/videoRearCam', '/dev/videoScienceCam', '/dev/video0', '/dev/video1']);
})
