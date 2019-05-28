function initRosWeb () {
  let ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  })

  ros.on('connection', function () {
    appendToConsole('Connected to websocket server.')
    checkTaskStatuses()
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
    appendToConsole(message.data)
  })

  /* rover commands */

  // setup a client for the rover_request service
  rover_request_client = new ROSLIB.Service({
    ros: ros,
    name: 'rover_request',
    serviceType: 'ArmRequest' // for now... might change
  })

  // setup a publisher for the arm_command topic
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
    // TODO: make sure the indices are correct... DONE?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?
    $('#right-front-rpm').text(message.velocity[0])
    $('#right-mid-rpm').text(message.velocity[1])
    $('#right-back-rpm').text(message.velocity[2])
    $('#left-front-rpm').text(message.velocity[3])
    $('#left-mid-rpm').text(message.velocity[4])
    $('#left-back-rpm').text(message.velocity[5])
  })

  // setup a subscriber for the rover_position topic
  rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'RoverPosition'
  })

  rover_position_listener.subscribe(function (message) {
    // TODO: place the data somewhere, call ros node, etc?
    // idk how this will work exactly, maybe a ros python node does stuff
    // this could still display the data though i guess
  })

  // setup a subscriber for the rover_feedback topic
  rover_feedback_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_feedback',
    messageType: 'std_msgs/String'
  })

  rover_feedback_listener.subscribe(function (message) {
    appendToConsole(message.data)
  })
}

/* functions used in main code */

function requestMuxChannel (elemID, callback) {
  let dev = elemID[elemID.length - 1]
  let request = new ROSLIB.ServiceRequest({ device: dev })
  let sentTime = new Date().getTime()

  console.log(request)
  if (dev != '?') {
    appendToConsole(
      'Sending request to switch to channel ' + $('a' + elemID).text()
    )
  }

  mux_select_client.callService(request, function (result) {
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

function requestTask (reqTask, reqStatus, buttonID, callback, reqArgs = '') {
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

  task_handler_client.callService(request, function (result) {
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
      appendToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-arm-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-arm-listener-btn')[0].checked = true
        }
      }
    })
    // check arm camera stream status
    requestTask('camera_stream', 2, '#toggle-arm-stream-btn', function (msgs) {
      appendToConsole(msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#toggle-arm-stream-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#toggle-arm-stream-btn')[0].checked = true
        }
      }
    })
  } else if (window.location.pathname == '/rover') {
    console.log('rover page')
  } else if (window.location.pathname == '/science') {
    console.log('science page')
    requestTask('science_listener', 2, '#science-listener-btn', function (msgs) {
      appendToConsole('msgs', msgs)
      if (msgs[0] && msgs.length == 2) {
        if (msgs[1].includes('not running')) {
          $('#science-listener-btn')[0].checked = false
        } else if (msgs[1].includes('running')) {
          $('#science-listener-btn')[0].checked = true
          // check if activated
          sendScienceRequest('active', function (msgs) {
            appendToConsole(msgs)

            // would also check if msgs[0] was true but science MCU responds
            // with the same message too many times so it is always false
            // this is not the case with the 'activated0' response which only appears once
            if (msgs[1].includes('activated1')) {
              $('#activate-science-btn')[0].checked = true
            }
          })
        }
      }
    })
  } /* else if (window.location.pathname == '/rover') { //pds
    console.log('rover page')
  } */
}

function sendArmCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  console.log(command)
  appendToConsole('Sending "' + cmd + '" to arm Teensy')
  arm_command_publisher.publish(command)
}

/*
// usage
sendArmRequest('this is a command', function (succeeded) {
  console.log(succeeded ? 'command succeeded' : 'command failed')
})
*/

function sendArmRequest (command, callback) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  science_request_client.callService(request, function (result) {
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

function sendScienceRequest (command, callback) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  science_request_client.callService(request, function (result) {
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

/*
// example usage
sendRoverRequest('this is a command', function (succeeded) {
  console.log(succeeded ? 'command succeeded' : 'command failed')
})
*/

function sendRoverRequest (command, callback) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command "' + command + '"')

  rover_request_client.callService(request, function (result) {
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

/*
returns the currently set ROS_MASTER_URI value

usage:
getRoverIP(function(callback) {})
*/
function getRoverIP (callback) {
  let request = new ROSLIB.ServiceRequest({ ping: 'rover_ip' })
  console.log('request', request)
  ping_client.callService(request, function (result) {
    let msg = result.response
    if (result.response) {
      callback(result.response)
    }
  })
}
