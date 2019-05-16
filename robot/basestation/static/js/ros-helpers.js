function initRosWeb(){
  let ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  })

  ros.on('connection', function() {
    appendToConsole('Connected to websocket server.')
  })

  ros.on('error', function(error) {
    appendToConsole('Error connecting to websocket server: ', error)
  })

  ros.on('close', function() {
    appendToConsole('Connection to websocket server closed.')
  })

  /* general controls */

  // setup a client for the mux_select service
  mux_select_client = new ROSLIB.Service({
    ros : ros, name : 'mux_select', serviceType : 'SelectMux'
  })

  // setup a client for the serial_cmd service
  serial_cmd_client = new ROSLIB.Service({
    ros : ros, name : 'serial_cmd', serviceType : 'SerialCmd'
  })

  // setup a client for the task_handler service
  task_handler_client = new ROSLIB.Service({
    ros : ros, name : 'task_handler', serviceType : 'HandleTask'
  })

  /* arm controls */

  // setup a client for the arm_request service
  arm_request_client = new ROSLIB.Service({
  	ros : ros, name : 'arm_request', serviceType : 'ArmRequest'
  });

  // setup a publisher for the arm_command topic
  arm_command_publisher = new ROSLIB.Topic({
      ros : ros, name : 'arm_command', messageType : 'std_msgs/String'
    });

  // setup a subscriber for the arm_joint_states topic
  arm_joint_states_listener = new ROSLIB.Topic({
    ros : ros, name : 'arm_joint_states', messageType : 'sensor_msgs/JointState'
  })

  arm_joint_states_listener.subscribe(function(message) {
    for (var angle in message.position) {
      //let motor = angle+1;
      let motor = String.fromCharCode(angle.charCodeAt(0) + 1)
      $('#m'+motor+'-angle').text(message.position[angle])
    }
  })

  // setup a subscriber for the arm_feedback topic
  arm_feedback_listener = new ROSLIB.Topic({
    ros : ros, name : 'arm_feedback', messageType : 'std_msgs/String'
  });

  /* rover controls (placeholder descriptions, names are subject to change) */

  // setup a client for the rover_request service

  // setup a publisher for the rover_command topic

  // setup a subscriber for the rover_vels topic

  // setup a subscriber for the gps_data topic

}

function requestMuxChannel(elemID) {
  let dev = elemID[elemID.length -1]
  let request = new ROSLIB.ServiceRequest({ device : dev })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to switch to channel ' + $('a'+elemID).text())
  scrollToBottom()

  mux_select_client.callService(request, function(result){
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response.slice(0, result.response.length-1) // remove newline character
    if (msg.includes('failed') || msg.includes('ERROR')) { // how to account for a lack of response?
      appendToConsole('Request failed. Received \"' + msg + '"')
    }
    else {
      $('button#mux').text('Device ' + $('a'+elemID).text())
      appendToConsole('Received \"' + msg + '\" with ' + latency.toString() + ' ms latency')
    }
    scrollToBottom()
  })
}

// todo: make sure that this doesn't work if the odroid's serial
// port is already open due to an already running python node
// (for example, roverlistener.py or ArmNode.py or whatever)
function requestSerialCommand(command) {
  let request = new ROSLIB.ServiceRequest({ msg : command+'\n' })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command \"' + command + '\"')
  scrollToBottom()

  mux_select_client.callService(request, function(result){
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response.slice(0, result.response.length-1) // remove newline character
    if (msg.includes('failed') || msg.includes('ERROR')) { // how to account for a lack of response?
      appendToConsole('Request failed. Received \"' + msg + '\"')
    }
    else {
      appendToConsole('Received \"' + msg + '\" with ' + latency.toString() + ' ms latency')
    }
    scrollToBottom()
  })
}

function requestTask(reqTask, reqStatus, buttonID) {
  let request = new ROSLIB.ServiceRequest({ task : reqTask, status : reqStatus })
  let sentTime = new Date().getTime()

  console.log(request)
  if (reqStatus == 0) {
    appendToConsole('Sending request to stop ' + reqTask + ' task')
  } else if (reqStatus == 1) {
    appendToConsole('Sending request to start ' + reqTask + ' task')
  }
  scrollToBottom()

  task_handler_client.callService(request, function(result){
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response.slice(0, result.response.length-1) // remove newline character
    if (msg.includes('Failed') || msg.includes('shutdown request') || msg.includes('unavailable')) { // how to account for a lack of response?
      appendToConsole('Request failed. Received \"' + msg + '\"')
      scrollToBottom()
      return false
    }
    else {
      appendToConsole('Received \"' + msg + '\" with ' + latency.toString() + ' ms latency')
      if (reqStatus == 0) {
        $(buttonID)[0].checked = false
      } else if (reqStatus == 1) {
        $(buttonID)[0].checked = true
      }
      scrollToBottom()
      return true
    }
  })
}

function sendArmCommand(cmd) {
  let command = new ROSLIB.Message({data : cmd+'\n'})
  console.log(command)
  appendToConsole('Sending \"' + cmd + '\" to arm Teensy')
  scrollToBottom()
  arm_command_publisher.publish(command);
}

function sendArmRequest(command) {
  let request = new ROSLIB.ServiceRequest({ msg : command })
  let sentTime = new Date().getTime()

  console.log(request)
  appendToConsole('Sending request to execute command \"' + command + '\"')
  scrollToBottom()

  arm_request_client.callService(request, function(result){
    let latency = millisSince(sentTime)
    console.log(result)
    let msg = result.response.slice(0, result.response.length-1) // remove newline character
    if (!result.success) { // how to account for a lack of response?
      appendToConsole('Request failed. Received \"' + msg + '\"')
    }
    else {
      appendToConsole('Received \"' + msg + '\" with ' + latency.toString() + ' ms latency')
    }
    scrollToBottom()
  })
}
