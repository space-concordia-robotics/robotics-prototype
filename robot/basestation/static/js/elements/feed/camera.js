$(document).ready(() => {
  // competition specific naming convention for port aliases for usb cameras
  // prefixes + suffixes
  const PORT_PREFIX = '/dev/video'
  const TOPIC_SUFFIX = 'Cam'
  const TOPIC_PREFIX = 'video'
  // rover usb port aliases
  const REAR_PORT = PORT_PREFIX + 'Rear'
  const FRONT_PORT = PORT_PREFIX + 'Front'
  const ARM_SCIENCE_PORT = PORT_PREFIX + 'ArmScience'
  // corresponding ROS topic names
  const REAR_STREAM_TOPIC = TOPIC_PREFIX + 'Rear' + TOPIC_SUFFIX
  const FRONT_STREAM_TOPIC = TOPIC_PREFIX + 'Front' + TOPIC_SUFFIX
  const ARM_SCIENCE_STREAM_TOPIC = TOPIC_PREFIX + 'ArmScience' + TOPIC_SUFFIX

  // task handler's recognized name for camera stream task
  const CAMERA_TASK = 'camera_stream'
  // response messages from task handler
  // these values must not be altered unless they are updated in the task handler server
  const CAMERA_START_SUCCESS_RESPONSE = 'Started camera stream'
  const CAMERA_STOP_SUCCESS_RESPONSE = 'Stopped camera stream'

  // path to stream offline image
  const STREAM_OFF = '../../../images/stream-offline.jpg'

  $('#front-camera-stream-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#front-camera-stream-btn').is(':checked')) {
      requestTask(
        CAMERA_TASK,
        STATUS_START,
        function (msgs) {
          printErrToConsole(msgs)
          console.log('front camera ON msgs:', msgs)
          if (msgs[1].includes(CAMERA_START_SUCCESS_RESPONSE)) {
            $('img#camera-feed')[0].src = getStreamURL(FRONT_STREAM_TOPIC)
            $('img#camera-feed').addClass('rotateimg180')
          } else {
            appendToConsole('Failed to open stream')
            $('#front-camera-stream-btn')[0].checked = false
          }
        },
        FRONT_PORT
      )
    } else {
      requestTask(
        CAMERA_TASK,
        STATUS_STOP,
        function (msgs) {
          printErrToConsole(msgs)
          console.log('front camera OFF msgs:', msgs)
          if (msgs[1].includes(CAMERA_STOP_SUCCESS_RESPONSE)) {
            $('img#camera-feed')[0].src = STREAM_OFF
            $('img#camera-feed').removeClass('rotateimg180')
          } else {
            appendToConsole('Failed to close stream')
          }
        },
        FRONT_PORT
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

  $('#local-stream-on-btn').on('click', function (event) {
    let localPort = $('#local-stream-select option:selected')[0].text
    let streamTopic = localPort.split('/')
    streamTopic = streamTopic[streamTopic.length - 1] + TOPIC_SUFFIX

    // click makes it checked during this time, so trying to enable
    requestTask(
      CAMERA_TASK,
      STATUS_START,
      function (msgs) {
        printErrToConsole(msgs)
        console.log('local camera ON msgs:', msgs)
        if (msgs[1].includes(CAMERA_START_SUCCESS_RESPONSE)) {
          $('img#camera-feed')[0].src = getStreamURL(streamTopic)
          console.log('asasdfasdf')
        } else {
          appendToConsole('Failed to open stream')
        }
      },
      localPort
    )
  })

  $('#local-stream-off-btn').on('click', function (event) {
    let localPort = $('#local-stream-select option:selected')[0].text

    requestTask(
      CAMERA_TASK,
      STATUS_STOP,
      function (msgs) {
        printErrToConsole(msgs)

        if (msgs[1].includes(CAMERA_STOP_SUCCESS_RESPONSE)) {
          console.log('local camera OFF msgs:', msgs)

          // succeeded to close stream
          $('img#camera-feed')[0].src = STREAM_OFF
        } else {
          // failed to close stream
          appendToConsole('Failed to close stream')
        }
      },
      localPort
    )
  })
})
