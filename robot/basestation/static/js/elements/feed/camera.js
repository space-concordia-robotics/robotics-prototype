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
  const STREAM_OFF = '../../static/img/camera/stream-offline.png'
  const POWER_ON = '../../static/img/camera/power_on.png';
  const POWER_OFF = '../../static/img/camera/power_off.png';

  function startCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_STREAM, STATUS_START, msgs => {
        if(msgs[0])
            successCallback();
        else
            printErrToConsole(msgs[1]);
    
    }, cameraStream);
  }

  function stopCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_STREAM, STATUS_STOP, msgs => {
        if(msgs[0])
            successCallback();
        else
            printErrToConsole(msgs[1]);
    
    }, cameraStream);
  }

  function setStreamSelection (availableStreams) {
    $('.camera-selections').children().empty();
    availableStreams.forEach(elt => {
      $('.camera-selections').append(
        '<li class="camera-selection-element">' + elt + '</li>'
      )
    })
  }

  function getStreamURL(topicName) {
    return 'http://' + getRoverIP() + ':8080/stream?topic=/' + topicName + '/image_raw'
  }

  function getCameraFilename(cameraPanel) {
    let cameraNameElement = cameraPanel.find(".camera-name");
    let cameraName = cameraNameElement.attr("stream");
    let cameraNameSplit = cameraName.split('/');
    let cameraFilename = cameraNameSplit[cameraNameSplit.length - 1]
    return cameraFilename;
  }

  function showStreamOn(cameraPanel) {
    let cameraFeed = cameraPanel.find(".camera-feed");
    let cameraPower = cameraPanel.find('.camera-power');

    cameraFeed.attr("src", getStreamURL(getCameraFilename(cameraPanel) + 'Cam'));
    cameraFeed.css("padding", "0px");

    cameraPower.attr("power-on", "true");
    cameraPower.attr("src", POWER_ON);
  }

  function showStreamOff(cameraPanel) {
    let cameraFeed = cameraPanel.find('.camera-feed');
    let cameraPower = cameraPanel.find('.camera-power');

    cameraFeed.attr("src", STREAM_OFF);
    cameraFeed.css("padding", "10px");
  
    cameraPower.attr("power-on", "false");
    cameraPower.attr("src", POWER_OFF);
  }

  $('.camera-screenshot').click(e => {
    let cameraPanel = $(e.target).parents('.camera-panel');
    console.log(cameraPanel);
    let cameraStreamName = getCameraFilename(cameraPanel) + 'Cam'
    $.ajax('/capture_image?stream_url=' + getStreamURL(cameraStreamName), {
      success: function (data) {
        appendToConsole(data.msg)
        if (!data.msg.includes('success')) {
          appendToConsole('Something went wrong, got', data.msg)
        } else {
          appendToConsole("Successfully saved screenshot")
        }
      },
      error: function () {
        appendToConsole('An error occured while taking a screenshot')
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

  $(document).on('click', '.camera-selection-element', e => {
    let selectedStream = $(e.target)[0]
    let cameraPanel = $(e.target).parents(".camera-panel");
    let cameraNameElement = cameraPanel.find(".camera-name");
    let cameraName = selectedStream.innerHTML.replace(/(\r\n|\n|\r)/gm, "");
    cameraNameElement.attr("stream", cameraName);
    let cameraNameSplit = cameraName.split("/");
    cameraNameElement[0].innerHTML = cameraNameSplit[cameraNameSplit.length - 1];

    requestTask(CAMERA_STREAM, STATUS_CHECK, msgs => {
      if(msgs[0])
      {
        let activePorts = msgs[1].replace(/(\r\n|\n|\r)/gm, "").split(',');
        if (activePorts.includes(cameraName))
        {
          showStreamOn(cameraPanel);
        }
      }
    });
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

  $('.camera-feed').on("error", (e) => {
    let cameraPanel = $(e.target).parents('.camera-panel');
    showStreamOff(cameraPanel);
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

  $('.camera-power').click((e) => {
    let cameraPower = $(e.target);
    let cameraPanel = cameraPower.parents(".camera-panel");
    let cameraNameElement = cameraPanel.find(".camera-name");
    let cameraName = cameraNameElement.attr("stream");

    if (cameraName == ""){
      appendToConsole("Please select a stream");
      return;
    }   

    let isPoweredOn = cameraPower.attr("power-on") == "true";
    if(isPoweredOn)
      stopCameraStream(cameraName, () => {
        showStreamOff(cameraPanel);
      });
    else
      startCameraStream(cameraName, () => {
        showStreamOn(cameraPanel);
      });
  }); 
})
