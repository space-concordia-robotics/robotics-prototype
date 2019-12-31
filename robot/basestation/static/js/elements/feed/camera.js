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
  const CAMERA_STREAM = 'camera_stream'
  const CAMERA_PORTS = 'camera_ports'
  // response messages from task handler
  // these values must not be altered unless they are updated in the task handler server
  const CAMERA_START_SUCCESS_RESPONSE = 'Started camera stream'
  const CAMERA_STOP_SUCCESS_RESPONSE = 'Stopped camera stream'

  // path to stream offline image
  const STREAM_OFF = '../../../images/stream-offline.jpg'

  function startCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_STREAM, STATUS_START, msgs => {
        if(msgs[0])
            successCallback();
        else
            printErrorToConsole(msgs[1]);
    
    }, cameraStream);
  }

  function stopCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_STREAM, STATUS_STOP, msgs => {
        if(msgs[0])
            successCallback();
        else
            printErrorToConsole(msgs[1]);
    
    }, cameraStream);
  }

  $('.camera-screenshot').on('click', function (event) {
    $.ajax('/capture_image', {
      success: function (data) {
        appendToConsole(data.msg)
        if (!data.msg.includes('success')) {
          appendToConsole('Something went wrong, got', data.msg)
        } else {
          appendToConsole("Successfully saved screenshot")
        }
      },
      error: function () {
        console.log('An error occured while taking a screenshot')
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
    $('.camera-selections').children().empty();
    availableStreams.forEach(elt => {
      $('.camera-selections').append(
        '<li class="camera-selection-element">' + elt + '</li>'
      )
    })
  }

  $(document).on('click', '.camera-selection-element', e => {
    let selectedStream = $(e.target)[0]
    let cameraPanel = $(e.target).parents(".camera-panel");
    let cameraNameElement = cameraPanel.find(".camera-name");
    let cameraName = selectedStream.innerHTML.replace(/(\r\n|\n|\r)/gm, "");
    console.log(cameraName);
    cameraNameElement.attr("stream", cameraName);
    let cameraNameSplit = cameraName.split("/");
    console.log(cameraNameSplit);
    cameraNameElement[0].innerHTML = cameraNameSplit[cameraNameSplit.length - 1];
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

  $(".camera-select").click((e) => {
    let cameraSelect = $(e.target);
    let menuOpened = cameraSelect.attr("aria-expanded") == "false";

    if(menuOpened)
    {
        requestTask(CAMERA_PORTS, STATUS_CHECK, msgs => {
          printErrToConsole(msgs);

          if(msgs[0])
            setStreamSelection(msgs[1].split(','));
          else
            setStreamSelection([]); 
        });
    }
  });

  $('.camera-power');
})
