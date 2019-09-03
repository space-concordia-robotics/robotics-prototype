REQUEST_TIMEOUT = 3000

function initRosWeb () {
  let ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  })
  ros.on('connection', function () {
    appendToConsole('Connected to websocket server.')
    checkTaskStatuses()
    if (window.location.pathname == '/rover') {
      initNavigationPanel()
    }
  })
  ros.on('error', function (error) {
    appendToConsole('Error connecting to websocket server: ', error)
  })
  ros.on('close', function () {
    appendToConsole('Connection to websocket server closed.')
  })

  /* general controls */

  // setup a client for the ping service
  ping_client = new ROSLIB.Service({
    ros: ros,
    name: 'ping_response',
    serviceType: 'PingResponse'
  })

  // setup a client for the mux_select service
  mux_select_client = new ROSLIB.Service({
    ros: ros,
    name: 'mux_select',
    serviceType: 'SelectMux'
  })
  // setup a client for the serial_cmd service
  serial_cmd_client = new ROSLIB.Service({
    ros: ros,
    name: 'serial_cmd',
    serviceType: 'SerialCmd'
  })
  // setup a client for the task_handler service
  task_handler_client = new ROSLIB.Service({
    ros: ros,
    name: 'task_handler',
    serviceType: 'HandleTask'
  })
  // setup a subscriber for the battery_voltage topic
  battery_voltage_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'battery_voltage',
    messageType: 'std_msgs/Float32'
  })
  battery_voltage_listener.subscribe(function (message) {
    $('#battery-voltage').text(message.data.toFixed(2))
  })

  /* arm controls */

  // setup a client for the arm_request service
  arm_request_client = new ROSLIB.Service({
    ros: ros,
    name: 'arm_request',
    serviceType: 'ArmRequest'
  })
  // setup a publisher for the arm_command topic
  arm_command_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'arm_command',
    messageType: 'std_msgs/String'
  })
  // setup a publisher for the ik_command topic
  ik_command_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'ik_command',
    messageType: 'IkCommand'
  })

  gripper_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'gripper_position',
    messageType: 'geometry_msgs/Point'
  })

  gripper_listener.subscribe(function (msg) {
    console.log('gripper_postion:', msg)
  })

  // setup a subscriber for the arm_joint_states topic
  arm_joint_states_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'arm_joint_states',
    messageType: 'sensor_msgs/JointState'
  })
  arm_joint_states_listener.subscribe(function (message) {
    for (var angle in message.position) {
      // let motor = angle+1;
      let motor = String.fromCharCode(angle.charCodeAt(0) + 1)
      $('#m' + motor + '-angle').text(message.position[angle])
    }
  })
  // setup a subscriber for the arm_feedback topic
  arm_feedback_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'arm_feedback',
    messageType: 'std_msgs/String'
  })
  arm_feedback_listener.subscribe(function (message) {
    appendToConsole(message.data)
  })

  /* science commands */

  // setup a client for the science_request service
  science_request_client = new ROSLIB.Service({
    ros: ros,
    name: 'science_request',
    serviceType: 'ScienceRequest'
  })

  // setup a subscriber for the arm_joint_states topic
  science_data_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'science_feedback',
    messageType: 'std_msgs/String'
  })

  science_data_listener.subscribe(function (message) {
    appendToConsole(message.data, true, false)
    // isActivated, drillDirection, elevatorDirection
    let dataKeyValues = message.data.replace('Science data:', '').split(',')
    let keys = dataKeyValues.map(i => {
      return i.split(':')[0]
    })
    let values = dataKeyValues.map(i => {
      return i.split(':')[1]
    })

    // isActivated
    if (values[0] == '0') {
      $('#activate-science-btn')[0].checked = false
    } else if (values[0] == '1') {
      $('#activate-science-btn')[0].checked = true
    }

    // drillDirection
    if (values[1] == '0') {
      lightUp('#cw-btn')
      greyOut('#ccw-btn')
    } else if (values[1] == '1') {
      lightUp('#ccw-btn')
      greyOut('#cw-btn')
    }

    // elevatorDirection
    if (values[2] == '0') {
      lightUp('#elevator-up-btn')
      greyOut('#elevator-down-btn')
    } else if (values[2] == '1') {
      lightUp('#elevator-down-btn')
      greyOut('#elevator-up-btn')
    }

    if (values[3] == '0') {
      $('#pump-dir-label').text('DIR: OUT')
      $('#pump-dir-toggle')[0].checked = false
    } else if (values[3] == '1') {
      $('#pump-dir-label').text('DIR: IN')
      $('#pump-dir-toggle')[0].checked = true
    }

    // photo resistor Voltage
    $('#photo-resistor-voltage').val(values[4])

    // LED 1 ON
    if (values[5] == '0') {
      $('#led1-toggle')[0].checked = false
    } else if (values[5] == '1') {
      $('#led1-toggle')[0].checked = true
    }

    // LED 2 ON
    if (values[6] == '0') {
      $('#led2-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#led2-toggle')[0].checked = true
    }

    // Vibrator ON statuses
    if (values[7] == '0') {
      $('#vibrator1-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator1-toggle')[0].checked = true
    }

    if (values[8] == '0') {
      $('#vibrator2-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator2-toggle')[0].checked = true
    }

    if (values[9] == '0') {
      $('#vibrator3-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator3-toggle')[0].checked = true
    }

    if (values[10] == '0') {
      $('#vibrator4-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator4-toggle')[0].checked = true
    }

    if (values[11] == '0') {
      $('#vibrator5-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator5-toggle')[0].checked = true
    }

    if (values[12] == '0') {
      $('#vibrator6-toggle')[0].checked = false
    } else if (values[6] == '1') {
      $('#vibrator6-toggle')[0].checked = true
    }

    $('#drill-rpm').val(values[13])
  })

  /* rover commands */

  // setup a client for the rover_request service
  rover_request_client = new ROSLIB.Service({
    ros: ros,
    name: 'rover_request',
    serviceType: 'ArmRequest' // for now... might change
  })
  // setup a publisher for the rover_command topic
  rover_command_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_command',
    messageType: 'std_msgs/String'
  })
  // setup a subscriber for the rover_joint_states topic
  rover_joint_states_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_joint_states',
    messageType: 'sensor_msgs/JointState'
  })
  rover_joint_states_listener.subscribe(function (message) {
    $('#right-front-rpm').text(message.velocity[0])
    $('#right-mid-rpm').text(message.velocity[1])
    $('#right-rear-rpm').text(message.velocity[2])
    $('#left-front-rpm').text(message.velocity[3])
    $('#left-mid-rpm').text(message.velocity[4])
    $('#left-rear-rpm').text(message.velocity[5])
  })
  // setup a subscriber for the rover_position topic
  rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'geometry_msgs/Point'
  })
  rover_position_listener.subscribe(function (message) {
    $('#rover-latitude').text(message.x)
    $('#rover-longitude').text(message.y)
    $('#rover-heading').text(message.z)
  })
  // setup a subscriber for the rover_twist topic
  rover_twist_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_twist',
    messageType: 'geometry_msgs/Twist'
  })
  rover_twist_listener.subscribe(function (message) {
    $('#rover-speed').text(message.linear.x)
    $('#rover-angular-velocity').text(message.angular.x)
  })
  // setup a subscriber for the rover_feedback topic
  rover_feedback_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_feedback',
    messageType: 'std_msgs/String'
  })
  rover_feedback_listener.subscribe(function (message) {
    appendToConsole(message.data, true, false)
  })
  // setup a subscriber for the antenna_goal topic
  antenna_goal_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'antenna_goal',
    messageType: 'geometry_msgs/Point'
  })
  antenna_goal_listener.subscribe(function (message) {
    $('#recommended-antenna-angle').text(parseFloat(message.x).toFixed(3))
    $('#distance-to-rover').text(parseFloat(message.y).toFixed(2))
  })
  // setup gps parameters for antenna directing
  antenna_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_latitude'
  })
  antenna_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_longitude'
  })
  antenna_start_dir = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_start_dir'
  })

  // setup a subscriber for the rover_goal topic
  rover_goal_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_goal',
    messageType: 'geometry_msgs/Point'
  })
  rover_goal_listener.subscribe(function (message) {
    $('#recommended-rover-heading').text(parseFloat(message.x).toFixed(3))
    $('#distance-to-goal').text(parseFloat(message.y).toFixed(2))
  })
  // setup gps parameters for rover goals
  goal_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_latitude'
  })
  goal_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_longitude'
  })
}

initRosWeb()

/* functions used in main code */
function requestMuxChannel (elemID, callback, timeout = REQUEST_TIMEOUT) {
  let dev = elemID[elemID.length - 1]
  let request = new ROSLIB.ServiceRequest({ device: dev })
  let sentTime = new Date().getTime()

  console.log(request)
  if (dev != '?') {
    appendToConsole(
      'Sending request to switch to channel ' + $('a' + elemID).text()
    )
  }

  let timer = setTimeout(function() {
      callback([false, elemID + " timeout after " + timeout/1000 + " seconds"])
  }, timeout)

  mux_select_client.callService(request, function (result) {
    clearTimeout(timer)
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response // .slice(0, result.response.length - 1) // remove newline character
    if (msg.includes('failed') || msg.includes('ERROR')) {
      // how to account for a lack of response?
      appendToConsole('Request failed. Received "' + msg + '"')
      callback([false, msg])
    } else if (msg.includes('Current:')) {
      // -2 since last character is newline, second last is actual channel
      $('button#mux').text('Device ' + $('a#mux-' + msg[msg.length - 2]).text())
    } else {
      $('button#mux').text('Device ' + $('a' + elemID).text())
      appendToConsole(
        'Received "' + msg + '" with ' + latency.toString() + ' ms latency'
      )
      callback([true])
    }
  })
}
function requestSerialCommand (command, callback) {
  let request = new ROSLIB.ServiceRequest({ msg: command + '\n' })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  mux_select_client.callService(request, function (result) {
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response // .slice(0, result.response.length - 1) // remove newline character
    if (msg.includes('failed') || msg.includes('ERROR')) {
      // how to account for a lack of response?
      appendToConsole('Request failed. Received "' + msg + '"')
      callback([false, msg])
    } else {
      appendToConsole(
        'Received "' + msg + '" with ' + latency.toString() + ' ms latency'
      )
      callback([true])
    }
  })
}
function requestTask (reqTask, reqStatus, buttonID, callback, reqArgs = '', timeout = REQUEST_TIMEOUT) {
  var request
  if (reqArgs == '') {
    request = new ROSLIB.ServiceRequest({ task: reqTask, status: reqStatus })
  } else {
    request = new ROSLIB.ServiceRequest({
      task: reqTask,
      status: reqStatus,
      args: reqArgs
    })
  }
  let sentTime = new Date().getTime()

  console.log('request:', request)
  if (reqStatus == 0) {
    appendToConsole('Sending request to stop ' + reqTask + ' task')
  } else if (reqStatus == 1) {
    appendToConsole('Sending request to start ' + reqTask + ' task')
  } else if (reqStatus == 2) {
    appendToConsole('Sending request to check ' + reqTask + ' task status')
  }

  let timer = setTimeout(function() {
      callback([false, reqTask + " timeout after " + timeout/1000 + " seconds"])
  }, timeout)

  task_handler_client.callService(request, function (result) {
    clearTimeout(timer)
    let latency = millisSince(sentTime)
    console.log('result:', result)
    let msg = result.response
    if (
      msg.includes('Failed') ||
      msg.includes('shutdown request') ||
      msg.includes('unavailable') ||
      msg.includes('is already running') ||
      msg.includes('not available')
    ) {
      // how to account for a lack of response?
      appendToConsole('Request failed. Received "' + msg + '"')
      callback([false, msg])
    } else {
      appendToConsole(
        'Received "' + msg + '" with ' + latency.toString() + ' ms latency'
      )

      if (reqStatus == 0) {
        $(buttonID)[0].checked = false
      } else if (reqStatus == 1) {
        $(buttonID)[0].checked = true
      } else if (reqStatus == 2) {
        if (msg.includes('not')) {
          $(buttonID)[0].checked = false
        } else {
          $(buttonID)[0].checked = true
        }
      }
      callback([true, msg])
    }
  })
}

function checkTaskStatuses () {
  // regardless of page we're on
  requestMuxChannel('?', function (currentChannel) {
    console.log('currentChannel', currentChannel)
  })

  if (window.location.pathname == '/') {
    // check arm listener status
    requestTask('arm_listener', 2, '#toggle-arm-listener-btn', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-arm-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-arm-listener-btn')[0].checked = true
        }
      }
    })
    // check all camera stream status
    requestTask('camera_stream', 2, '#arm-science-camera-stream-btn', function (
      msgs
    ) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyArmScienceCam')
        ) {
          $('#camera-feed').removeClass('rotateimg180')
          $('#arm-science-camera-stream-btn')[0].checked = true
          $('#rear-camera-stream-btn')[0].checked = false
          $('#front-camera-stream-btn')[0].checked = false
        } else if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyFrontCam')
        ) {
          $('#camera-feed').addClass('rotateimg180')
          $('#front-camera-stream-btn')[0].checked = true
          $('#arm-science-camera-stream-btn')[0].checked = false
          $('#rear-camera-stream-btn')[0].checked = false
        } else if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyRearCam')
        ) {
          $('#camera-feed').addClass('rotateimg180')
          $('#rear-camera-stream-btn')[0].checked = true
          $('#front-camera-stream-btn')[0].checked = false
          $('#arm-science-camera-stream-btn')[0].checked = false
        }
      }
    })
  } else if (window.location.pathname == '/rover') {
    // check rover listener status
    requestTask('rover_listener', 2, '#toggle-rover-listener-btn', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-rover-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-rover-listener-btn')[0].checked = true
        }
      }
    })

    sendRequest("Rover", 'who', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[1].includes('Happy')) {
        $('#activate-rover-btn')[0].checked = true
      } else {
        $('#activate-rover-btn')[0].checked = false
      }
    })

    // initialize rover to open-loop mode
    sendRequest('Rover', 'open-loop', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[1].includes('loop status is: Open')) {
        appendToConsole('Open loop active')
        $('#toggle-rover-pid-btn')[0].checked = false
      } else {
        appendToConsole('Failed to activate open loop mode')
      }
    })


    // check all camera stream status
    requestTask('camera_stream', 2, '#arm-science-camera-stream-btn', function (
      msgs
    ) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyArmScienceCam')
        ) {
          $('#front-camera-stream-btn').removeClass('rotateimg180')
          $('#arm-science-camera-stream-btn')[0].checked = true
          $('#rear-camera-stream-btn')[0].checked = false
          $('#front-camera-stream-btn')[0].checked = false
        } else if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyFrontCam')
        ) {
          $('#front-camera-stream-btn').addClass('rotateimg180')
          $('#front-camera-stream-btn')[0].checked = true
          $('#arm-science-camera-stream-btn')[0].checked = false
          $('#rear-camera-stream-btn')[0].checked = false
        } else if (
          msgs[1].includes('running') &&
          msgs[1].includes('/dev/ttyRearCam')
        ) {
          $('#front-camera-stream-btn').removeClass('rotateimg180')
          $('#rear-camera-stream-btn')[0].checked = true
          $('#front-camera-stream-btn')[0].checked = false
          $('#arm-science-camera-stream-btn')[0].checked = false
        }
      }
    })
  } else if (window.location.pathname == '/science') {
    console.log('science page')
    requestTask('science_listener', 2, '#science-listener-btn', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#science-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#science-listener-btn')[0].checked = true
        }
      }
    })
  } /* else if (window.location.pathname == '/rover') { //pds
    console.log('rover page')
  } */
}

function sendIKCommand () {
  let command = new ROSLIB.Message({ data: cmd })
  console.log(command)
  appendToConsole('Sending "' + cmd + '" to IK node')
  ik_command_publisher.publish(cmd)
}

function sendArmCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  console.log(command)
  appendToConsole('Sending "' + cmd + '" to arm Teensy')
  arm_command_publisher.publish(command)
}

function sendRequest(device, command, callback, timeout = REQUEST_TIMEOUT) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  let timer = setTimeout(function() {
      callback([false, command + " timeout after " + timeout/1000 + " seconds"])
  }, timeout)

  var requestClient;
  switch(device)
  {
    case "Arm":
      requestClient = arm_request_client
    break
    case "Rover":
      requestClient = rover_request_client
    break
    case "Science":
      requestClient = science_request_client
    break
  }

  requestClient.callService(request, function (result) {

    clearTimeout(timer)
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response // .slice(0, result.response.length - 1) // remove newline character
    if (!result.success) {
      // how to account for a lack of response?
      appendToConsole('Request failed. Received "' + msg + '"')
      // return false
      callback([false, msg])
    } else {
      appendToConsole(
        'Received "' + msg + '" with ' + latency.toString() + ' ms latency'
      )
      // return true
      callback([true, msg])
    }
  })
}

function sendRoverCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  console.log(command)
  appendToConsole('Sending "' + cmd + '" to rover Teensy')
  rover_command_publisher.publish(command)
}

function initNavigationPanel () {
  let hasAntennaParams = true
  antenna_latitude.get(function (val) {
    if (val != null) {
      $('#antenna-latitude').text(val)
      antenna_longitude.get(function (val) {
        if (val != null) {
          $('#antenna-longitude').text(val)
          antenna_start_dir.get(function (val) {
            if (val != null) {
              $('#antenna-start-dir').text(val)
              appendToConsole(
                'Antenna goal parameters already set, displaying antenna directions'
              )
              $('#antenna-inputs').hide()
              $('#antenna-unchanging').show()
            } else {
              appendToConsole(
                'One or more antenna parameters is missing, if you would like antenna directions then please input them'
              )
              $('#antenna-inputs').show()
              $('#antenna-unchanging').hide()
            }
          })
        } else {
          appendToConsole(
            'One or more antenna parameters is missing, if you would like antenna directions then please input them'
          )
          $('#antenna-inputs').show()
          $('#antenna-unchanging').hide()
        }
      })
    } else {
      appendToConsole(
        'One or more antenna parameters is missing, if you would like antenna directions then please input them'
      )
      $('#antenna-inputs').show()
      $('#antenna-unchanging').hide()
    }
  })

  goal_latitude.get(function (val) {
    if (val != null) {
      $('#goal-latitude').text(val)
      goal_longitude.get(function (val) {
        if (val != null) {
          appendToConsole(
            'GPS goal parameters already set, displaying directions to the goal'
          )
          $('#goal-longitude').text(val)
          $('#goal-inputs').hide()
          $('#goal-unchanging').show()
        } else {
          appendToConsole(
            'One or more GPS goal parameters is missing, if you would like directions to the goal then please input them'
          )
          $('#goal-inputs').show()
          $('#goal-unchanging').hide()
        }
      })
    } else {
      appendToConsole(
        'One or more GPS goal parameters is missing, if you would like directions to the goal then please input them'
      )
      $('#goal-inputs').show()
      $('#goal-unchanging').hide()
    }
  })
}

/*
returns the currently set ROS_MASTER_URI value

usage:
getRoverIP(function(callback) { // do something with the response })
*/
function getRoverIP (callback) {
  /*
  let request = new ROSLIB.ServiceRequest({ ping: 'rover_ip' })
  console.log('request', request)
  ping_client.callService(request, function (result) {
    let msg = result.response
    if (result.response) {
      callback(result.response)
    }
  })
  */
  return '172.16.1.30' // competition
  // return '127.0.0.1' // local
}
