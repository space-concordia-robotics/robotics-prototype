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
  const CAMERA_PORTS = 'camera_ports'
  // response messages from task handler
  // these values must not be altered unless they are updated in the task handler server
  const CAMERA_START_SUCCESS_RESPONSE = 'Started camera stream'
  const CAMERA_STOP_SUCCESS_RESPONSE = 'Stopped camera stream'

  // path to stream offline image
  const STREAM_OFF = '../../static/img/camera/stream_offline.png'
  const POWER_ON = '../../static/img/camera/power_on.png'
  const POWER_OFF = '../../static/img/camera/power_off.png'

  const RECORDING_ON = '../../../static/img/camera/record_on.png'
  const RECORDING_OFF = '../../../static/img/camera/record_off.png'

  function getStreamURL(topicName) {
    return 'http://' + getRoverIP() + ':8080/stream?topic=/' + topicName + '/image_raw'
  }

  function getCameraFilename(cameraPanel) {
    let cameraName = getCameraName(cameraPanel)
    let cameraNameSplit = cameraName.split('/')
    let cameraFilename = cameraNameSplit[cameraNameSplit.length - 1]
    return cameraFilename
  }

  function getCameraName(cameraPanel){
    let cameraNameElement = cameraPanel.find('.camera-name')
    let cameraName = cameraNameElement.attr('stream')
    return cameraName
  }

  function startCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_TASK, STATUS_START, msgs => {
        if(msgs[0])
        {
            successCallback()
        }
        else
        {
            printErrToConsole(msgs[1])
        }

    }, cameraStream)
  }

  function stopCameraStream(cameraStream, successCallback = () => {}) {
    requestTask(CAMERA_TASK, STATUS_STOP, msgs => {
        if(msgs[0])
        {
            successCallback()
        }
        else
        {
            printErrToConsole(msgs[1])
        }

    }, cameraStream)
  }

  function startRecording(stream_url, callback = () => {}) {
    const start_recording_url = '/initiate_feed_recording?stream_url=' + stream_url
    $.ajax(start_recording_url, {
      success: data => {
        appendToConsole(data.msg)
        callback(data)
      },
      error: (jqXHR, exception) => {
        flaskError(jqXHR, exception, start_recording_url)
      }
    })
  }

  function stopRecording(stream_url, callback = () => {}) {
    const stop_recording_url = '/stop_feed_recording?stream_url=' + stream_url
    $.ajax(stop_recording_url, {
      success: data => {
        appendToConsole(data.msg)
        callback(data)
      },
      error: (jqXHR, exception) => {
        flaskError(jqXHR, exception, stop_recording_url)
      }
    })
  }

  function isRecording(stream_url, callback = () => {}) {
    const is_recording_url = 'is_recording?stream_url=' + stream_url
    $.ajax(is_recording_url, {
      success: data => {
        callback(data.is_recording)
      },
      error: (jqXHR, exception)  => {
        flaskError(jqXHR, exception, is_recording_url)
      }
    })
  }

  function setStreamSelection (availableStreams) {
    $('.camera-selections').children().empty()
    availableStreams.forEach(elt => {
      $('.camera-selections').append(
        '<li class=\'camera-selection-element\'>' + elt + '</li>'
      )
    })
  }

  function showStreamOn(cameraPanel) {
    let cameraFeed = cameraPanel.find('.camera-feed')
    let cameraPower = cameraPanel.find('.camera-power')
    let cameraControls = cameraPanel.find('.camera-controls')

    let cameraName = getCameraFilename(cameraPanel) + TOPIC_SUFFIX

    cameraFeed.attr('src', getStreamURL(cameraName))
    cameraFeed.css('padding', '0px')

    cameraPower.attr('power-on', 'true')
    cameraPower.attr('src', POWER_ON)

    cameraControls.attr('camera-name', cameraName)
    console.log("memes")
  }

  function showStreamOff(cameraPanel) {
    let cameraFeed = cameraPanel.find('.camera-feed')
    let cameraPower = cameraPanel.find('.camera-power')

    cameraFeed.attr('src', STREAM_OFF)
    cameraFeed.css('padding', '10px')

    cameraPower.attr('power-on', 'false')
    cameraPower.attr('src', POWER_OFF)
  }

  $('.camera-screenshot').click(e => {
    let cameraPanel = $(e.target).parents('.camera-panel')
    let cameraNameElement = cameraPanel.find('.camera-name')
    let cameraName = cameraNameElement.attr('stream')
    let cameraStreamName = getCameraFilename(cameraPanel) + TOPIC_SUFFIX
    let cameraPower = cameraPanel.find('.camera-power')
    let rotation = cameraPanel.attr("rotation")


    if (getCameraName(cameraPanel) == "" || cameraPower.attr("power-on") == "false"){
      appendToConsole("Please turn on a stream")
      return
    }

    $.ajax('/capture_image?stream_url=' + getStreamURL(cameraStreamName) + '&camera_rotation=' + rotation, {
      success: function (data) {
        appendToConsole(data.msg)

        },
      error: function () {
        appendToConsole('An error occured while taking a screenshot')
      }
    })
  })

  // Rotate camera feed 90 degrees CCW
  $('.camera-rotl').click(e => {
    let cameraPanel = $(e.target).parents(".camera-panel")
    let rotation = cameraPanel.attr("rotation")
    rotation = negativeModulo(--rotation, 4)
    cameraPanel.attr("rotation", rotation)
    let angle = rotation * 90
    let cameraFeed = cameraPanel.find(".camera-feed")
    rotateElement(cameraFeed, angle)
  })

  // Rotate camera feed 90 degrees CW
  $('.camera-rotr').click(e => {
    let cameraPanel = $(e.target).parents(".camera-panel")
    let rotation = cameraPanel.attr("rotation")
    rotation = negativeModulo(++rotation, 4)
    cameraPanel.attr("rotation", rotation)
    let angle = rotation * 90
    let cameraFeed = cameraPanel.find(".camera-feed")
    rotateElement(cameraFeed, angle)
  })

  $(document).on('click', '.camera-selection-element', e => {
    let selectedStream = $(e.target)[0]
    let cameraPanel = $(e.target).parents(".camera-panel")
    let cameraNameElement = cameraPanel.find(".camera-name")
    let cameraName = selectedStream.innerHTML.replace(/(\r\n|\n|\r)/gm, "")
    cameraNameElement.attr("stream", cameraName)
    let cameraNameSplit = cameraName.split("/")
    let cameraShortName = cameraNameSplit[cameraNameSplit.length - 1]
    cameraNameElement[0].innerHTML = cameraShortName

    let cameraPower = cameraPanel.find('.camera-power')
    cameraPower.attr("power-on", "false")
    cameraPower.attr("src", POWER_OFF)

    let cameraControls = cameraPanel.find('.camera-controls')
    cameraControls.attr("camera-name", cameraShortName + 'Cam')
    updateRecordingButton(cameraPanel, false)

    requestTask(CAMERA_TASK, STATUS_CHECK, msgs => {
      if(msgs[0])
      {
        let activePorts = msgs[1].replace(/(\r\n|\n|\r)/gm, "").split(',')
        if (activePorts.includes(cameraName))
        {
          showStreamOn(cameraPanel)
          let streamURL = getStreamURL(getCameraName(cameraPanel) + TOPIC_SUFFIX)
          isRecording(streamURL, is_recording => {
            updateRecordingButton(cameraPanel, is_recording)
          })
        }
        else
        {
          showStreamOff(cameraPanel)
        }
      }
    })
  })

  function updateRecordingButton(cameraPanel, isRecording)
  {
    let cameraRecording = cameraPanel.find('.camera-recording')
    cameraRecording.attr("recording", isRecording ? "true" : "false")
    cameraRecording.attr("src", isRecording ? RECORDING_ON : RECORDING_OFF)
  }

  $(".camera-stream").hover(inEvent => {
    let cameraStream = $(inEvent.currentTarget)
    let cameraControls = cameraStream.children(".camera-controls")
    cameraControls.css("display", "block")

  }, outEvent => {
    let cameraStream = $(outEvent.currentTarget)
    let cameraControls = cameraStream.children(".camera-controls")
    cameraControls.css("display", "none")
  })

  $('.camera-feed').on("error", (e) => {
    let cameraPanel = $(e.target).parents('.camera-panel')
    showStreamOff(cameraPanel)
  })

  $(".camera-select").click((e) => {
    let cameraSelect = $(e.target)
    let menuOpened = cameraSelect.attr("aria-expanded") == "false"

    if(menuOpened)
    {
      requestTask(CAMERA_PORTS, STATUS_CHECK, msgs => {
        printErrToConsole(msgs)

        if(msgs[0])
          setStreamSelection(msgs[1].split(','))
        else
          setStreamSelection([])
      })
    }
  })

  $('.camera-power').click((e) => {
    let cameraPower = $(e.target)
    let cameraPanel = cameraPower.parents(".camera-panel")
    let cameraName = getCameraName(cameraPanel)

    if (cameraName == ""){
      appendToConsole("Please select a stream")
      return
    }

    let streamURL = getStreamURL(getCameraFilename(cameraPanel) + TOPIC_SUFFIX)
    let isPoweredOn = cameraPower.attr("power-on") == "true"
    if(isPoweredOn)
      isRecording(streamURL, is_recording => {
        if(is_recording)
        {
          stopRecording(streamURL, (response) => {
            if(response.success)
            {
              updateRecordingButton(cameraPanel, false)
              stopCameraStream(cameraName, () => {
                showStreamOff(cameraPanel)
              })
            }
          })
        }
        else
        {
          stopCameraStream(cameraName, () => {
            showStreamOff(cameraPanel)
          })
        }
      })
    else
      startCameraStream(cameraName, () => {
        showStreamOn(cameraPanel)
      })
  })

  $('.camera-recording').click((e) => {
    let cameraRecording = $(e.target)
    let cameraPanel = cameraRecording.parents('.camera-panel')
    let cameraPower = cameraPanel.find('.camera-power')
    let cameraName = getCameraName(cameraPanel)

    if (getCameraName(cameraPanel) == "" || cameraPower.attr("power-on") == "false"){
      appendToConsole("Please turn on a stream")
      return
    }

    let streamURL = getStreamURL(getCameraFilename(cameraPanel) + TOPIC_SUFFIX)

    let isRecording = cameraRecording.attr("recording") == "true"

    if(isRecording)
    {
      stopRecording(streamURL, (response) => {
        if(response.success)
          updateRecordingButton(cameraPanel, false)
      })
    }
    else
    {
      startRecording(streamURL, (response) => {
        if(response.success)
          updateRecordingButton(cameraPanel, true)
      })
    }
  })
})
