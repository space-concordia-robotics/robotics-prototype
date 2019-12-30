REQUEST_TIMEOUT = 3000
ROTATE_TIMEOUT = 1000
lastRotate = 0

function initRosWeb () {
  ros = new ROSLIB.Ros({
    url: 'ws://' + env.HOST_IP + ':9090'
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

  // setup a subscriber for the battery_voltage topic
  battery_voltage_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'battery_voltage',
    messageType: 'std_msgs/Float32'
  })
  battery_voltage_listener.subscribe(function (message) {
    $('#battery-voltage').text(message.data.toFixed(2))
  })
  // setup a subscriber for the battery_temps topic
  battery_temps_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'battery_temps',
    messageType: 'geometry_msgs/Point'
  })
  battery_temps_listener.subscribe(function (message) {
    $('#battery-temp-1').text(parseFloat(message.x).toFixed(2))
    $('#battery-temp-2').text(parseFloat(message.y).toFixed(2))
    $('#battery-temp-3').text(parseFloat(message.z).toFixed(2))
  })
  // setup a subscriber for the wheel_motor_currents topic
  wheel_motor_currents_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'wheel_motor_currents',
    messageType: 'sensor_msgs/JointState'
  })
  wheel_motor_currents_listener.subscribe(function (message) {
    $('#right-front-current').text(parseFloat(message.effort[0]).toFixed(3))
    $('#right-mid-current').text(parseFloat(message.effort[1]).toFixed(3))
    $('#right-rear-current').text(parseFloat(message.effort[2]).toFixed(3))
    $('#left-front-current').text(parseFloat(message.effort[3]).toFixed(3))
    $('#left-mid-current').text(parseFloat(message.effort[4]).toFixed(3))
    $('#left-rear-current').text(parseFloat(message.effort[5]).toFixed(3))
  })
}

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

  let timer = setTimeout(function () {
    callback([false, elemID + ' timeout after ' + timeout / 1000 + ' seconds'])
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

STATUS_STOP=0
STATUS_START=1
STATUS_CHECK=2

/**
 * Sends a request to the task handler.
 * 
 * STATUS_STOP stops the task
 * STATUS_START starts the task
 * STATUS_CHECK obtains insight on the task
 * 
 * @reqTask The task related to the request
 * @reqStatus The status of the request
 * @callback Callback on success
 * @reqArgs Arguments of the request
 * @timeout Timeout of the request
 */
function requestTask (
  reqTask,
  reqStatus,
  callback,
  reqArgs = '',
  timeout = REQUEST_TIMEOUT
) {

  let request = new ROSLIB.ServiceRequest({ task: reqTask, status: reqStatus, args: reqArgs })
  let sentTime = new Date().getTime()

  if (reqStatus == STATUS_STOP) {
    appendToConsole('Sending request to stop ' + reqTask + ' task')
  } else if (reqStatus == STATUS_START) {
    appendToConsole('Sending request to start ' + reqTask + ' task')
  } else if (reqStatus == STATUS_CHECK) {
    appendToConsole('Sending request to check ' + reqTask + ' task status')
  }

  let timer = setTimeout(function () {
    callback([false, reqTask + ' timeout after ' + timeout / 1000 + ' seconds'])
  }, timeout)

  task_handler_client.callService(request, function (result) {
    clearTimeout(timer)
    let latency = millisSince(sentTime)
    let msg = result.response
    if (
      msg.includes('Failed') ||
      msg.includes('shutdown request') ||
      msg.includes('unavailable') ||
      msg.includes('is already running') ||
      msg.includes('not available')
    ) {
      appendToConsole('Request failed. Received "' + msg + '"')
      callback([false, msg])
    } else {
      appendToConsole(
        'Received "' + msg + '" with ' + latency.toString() + ' ms latency'
      )

      callback([true, msg])
    }
  })
}

function checkTaskStatuses () {
  if ($('#camera-local-mode-btn').length >= 1) {
    let isLocal = getCookie('localCameras')

    if (isLocal == '1') {
      $('#camera-local-mode-btn')[0].checked = true
    } else {
      $('#camera-local-mode-btn')[0].checked = false
    }
  }
  // regardless of page we're on
  requestMuxChannel('?', function (currentChannel) {
    console.log('currentChannel', currentChannel)
  })

  if (window.location.pathname == '/') {
    // check arm listener status
    requestTask('arm_listener', STATUS_CHECK, (msgs) => {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-arm-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-arm-listener-btn')[0].checked = true
        }
      }
    })
  } else if (window.location.pathname == '/science') {
    requestTask('science_listener', STATUS_CHECK, '#science-listener-btn', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#science-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#science-listener-btn')[0].checked = true
        }
      }
    })
  } else if (window.location.pathname == '/pds') {
    // check rover listener status
    requestTask('pds_listener', STATUS_CHECK, '#toggle-pds-listener-btn', function (msgs) {
      printErrToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-pds-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-pds-listener-btn')[0].checked = true
        }
      }
    })
  }
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

function sendRequest (device, command, callback, timeout = REQUEST_TIMEOUT) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  let timer = setTimeout(function () {
    callback([false, command + ' timeout after ' + timeout / 1000 + ' seconds'])
  }, timeout)

  var requestClient
  switch (device) {
    case 'Arm':
      requestClient = arm_request_client
      break
    case 'Rover':
      requestClient = rover_request_client
      break
    case 'Science':
      requestClient = science_request_client
      break
    case 'PDS':
      requestClient = pds_request_client
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

function sendPdsCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  console.log(command)
  appendToConsole('Sending "' + cmd + '" to PDS Teensy')
  pds_command_publisher.publish(command)
}

/*
returnsthe IP portion of the currntly set ROS_MASTER_URI
*/
function getRoverIP (callback) {
  console.log('roverIP: ' + env.ROS_MASTER_IP)
  console.log('hostIP: ' + env.HOST_IP)
  return env.ROS_MASTER_IP
}
