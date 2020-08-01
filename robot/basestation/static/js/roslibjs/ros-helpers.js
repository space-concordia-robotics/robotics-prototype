REQUEST_TIMEOUT = 3000
ROTATE_TIMEOUT = 1000
lastRotate = 0

// minimum and maximum acceptable battery voltages (volts)
MIN_VOLTAGE = 12.5
MAX_VOLTAGE = 16.8
VOLTAGE_LEEWAY = 0.5
VOLTAGE_WARNING = 1
// minimum and maximum acceptable battery tempratures (degrees celcius)
MIN_TEMP = 0
MAX_TEMP = 85
TEMP_LEEWAY = 1
TEMP_WARNING = 10
// variable to toggle unacceptable voltage and temperature indicators
ALERT_ENABLE = true

// ROS logging severity level constants
const ROSDEBUG = 1 // debug level
const ROSINFO = 2  // general level
const ROSWARN = 4  // warning level
const ROSERROR = 8 // error level
const ROSFATAL = 16 // fatal/critical level


// logs below this level will not be published or printed. Issue #202 will allow the user to set this value
const MINIMUM_LOG_LEVEL = ROSINFO

function initRosWeb () {
  ros = new ROSLIB.Ros({
    url: 'ws://' + env.HOST_IP + ':9090'
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

  /* Logging */

  // publishes log messages to /rosout
  ros_logger = new ROSLIB.Topic({
    ros: ros,
    name: 'rosout',
    messageType: 'rosgraph_msgs/Log'
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
  serial_cmd_client = new ROSLIB.Service({ ros: ros, name: 'serial_cmd',
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

  // setup a subscriber for the joint_states topic
  arm_joint_states_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_states',
    messageType: 'sensor_msgs/JointState'
  })
  arm_joint_states_listener.subscribe(function (message) {
    console.log(message)
    for (var angle in message.position) {
      // let motor = angle+1;
      let motor = String.fromCharCode(angle.charCodeAt(0) + 1)
      $('#m' + motor + '-angle').text(((180/Math.PI) * message.position[angle]).toFixed(2))
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

  // setup a publisher for the pds_command topic
  pds_command_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'pds_command',
    messageType: 'std_msgs/String'
  })

  // setup a publisher for the science_command topic
  science_command_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'science_command',
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
    // sets voltage to two decimal points
    let voltage = message.data.toFixed(2)
    $('#battery-voltage').text(voltage)

    // if statement to control voltage indicator switching between acceptable(white) and unacceptable(red)
    if ((voltage > MAX_VOLTAGE || voltage < MIN_VOLTAGE)) {
      textColor('#battery-voltage', 'red')

      if ($('#battery-voltage').attr('acceptable') === '1' && ALERT_ENABLE) {
        $('#battery-voltage').attr('acceptable', '0')
        errorSound()
        if (voltage > MAX_VOLTAGE) {
          navModalMessage('Warning: Voltage too high', 'Disconnect Battery first and then the BMS, and then discharge the Battery to 16.8V')
        } else if (voltage < MIN_VOLTAGE){
          navModalMessage('Warning: Voltage too low', 'Turn rover off, disconnect Battery and BMS, and then charge battery to 16.8V')
        }
      }
    } else if (voltage > MAX_VOLTAGE - VOLTAGE_LEEWAY || voltage < MIN_VOLTAGE + VOLTAGE_LEEWAY){
      if ($('#battery-voltage').attr('acceptable') === '0') {
        textColor('#battery-voltage', 'red')
      }

    } else {
      if (voltage < MIN_VOLTAGE + VOLTAGE_WARNING) {
        textColor('#battery-voltage', 'orange')

      } else {
        textColor('#battery-voltage', 'white')
      }

      if ($('#battery-voltage').attr('acceptable') === '0') $('#battery-voltage').attr('acceptable', '1')
    }
  })
  // setup a subscriber for the battery_temps topic
  battery_temps_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'battery_temps',
    messageType: 'geometry_msgs/Point'
  })
  battery_temps_listener.subscribe(function (message) {
    // sets temperatures to two decimal points
    let temps = [
      parseFloat(message.x).toFixed(2),
      parseFloat(message.y).toFixed(2),
      parseFloat(message.z).toFixed(2)
    ]

    $('.battery-temp').each(function(i, obj) {
      let $obj = $(obj)
      let temperature = temps[i]
      $obj.text(temperature)

      if ((temperature > MAX_TEMP || temperature < MIN_TEMP)) {
        $obj.css({'color': 'red'})

        if ($obj.attr('acceptable') === '1' && ALERT_ENABLE) {
          $obj.attr('acceptable', '0')
          errorSound()
          if (temperature > MAX_TEMP) {
            navModalMessage('Warning: Battery temperature (' + $obj.attr('sensorName') + ') too high.', 'Decrease temperature')
          } else if (temperature < MIN_TEMP) {
            navModalMessage('Warning: Battery temperature (' + $obj.attr('sensorName') + ') too low.', 'Increase temperature')
          }
        }
      } else if (temperature > MAX_TEMP - TEMP_LEEWAY || temperature < MIN_TEMP + TEMP_LEEWAY){
        if ($obj.attr('acceptable') === '0') {
          $obj.css({'color': 'red'})
        } else {
          $obj.css({'color': 'orange'})
        }

      } else {
        if (temperature > MAX_TEMP - TEMP_WARNING || temperature < MIN_TEMP + TEMP_WARNING) {
          $obj.css({'color': 'orange'})

        } else {
          $obj.css({'color': 'white'})
        }

        if ($obj.attr('acceptable') === '0') $obj.attr('acceptable', '1')
      }
    });

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

  // setup a subcriber function for rover_position topic
  rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'geometry_msgs/Point'
  })
}

/* functions used in main code */

/**
 * Gets the current time and formats it for ROS and the GUI.
 * @return array of two variables: ROS time object, string containing hh:mm:ss
*/
function rosTimestamp () {
  // next few lines taken and adjusted from roslibjs action server example
  let currentTime = new Date()
  let secs = currentTime.getTime()/1000 // seconds before truncating the decimal
  let secsFloored = Math.floor(secs) // seconds after truncating
  let nanoSecs = Math.round(1000000000*(secs-secsFloored)) // nanoseconds since the previous second

  // return a dictionary for the ROS log and a string for the gui console
  let stampTime = currentTime.toString().split(' ')[4] // hh:mm:ss from date object
  return [{secs : secsFloored, nsecs : nanoSecs}, stampTime]
}

/**
 * Creates and publishes a ROS log message to /rosout based on the parameters
 * @param logLevel one of the ROS loglevel constants defined above
 * @param timestamp ros time object containing seconds and nanoseconds
 * @param message the log message
*/
function publishRosLog (logLevel, timestamp, message) {
  ros_logger.publish(
    new ROSLIB.Message({
      // I'm only publishing the essentials. We could include more info if so desired
      header : {
        // seq // uint32: sequence ID, seems to increment automatically
        stamp : timestamp // dictionary: contains truncated seconds and nanoseconds
        // frame_id // string: probably only useful for tf
      },
      level : logLevel, // int: see log level constants above
      // name : '/web_gui', // name of the node (proposed)
      msg : message, // string: this is the log message
      // file // string: we could specify the js file that generated the log
      // function // string: we could specify the parent function that generated the log
      // line // uint32: we could specify the specific line of code that generated the log
      // topics // string[]: topic names that the node publishes
    })
  )
}

/**
 * Prints the log message to the GUI and chrome console.
 * Also calls publishRosLog with the message and parameters.
 * @param logLevel one of the ROS loglevel constants defined above
 * @param message the log message
*/
function rosLog (logLevel, message, devConsole = true, guiConsole = true) {
  logData = {}
  logData[ROSDEBUG] = {prefix : '[DEBUG]', type : 'log'}
  logData[ROSINFO] = {prefix : '[INFO]', type : 'log'}
  logData[ROSWARN] = {prefix : '[WARN]', type : 'warn'}
  logData[ROSERROR] = {prefix : '[ERROR]', type : 'error'}
  logData[ROSFATAL] = {prefix : '[FATAL]', type : 'error'}

  stamps = rosTimestamp()
  consoleMsg = logData[logLevel].prefix + ' [' + stamps[1] + ']: ' + message

  if (devConsole) {
    if (logData[logLevel].type === 'log') {
      console.log(consoleMsg)
    } else if (logData[logLevel].type === 'warn') {
      console.warn(consoleMsg)
    } else if (logData[logLevel].type === 'error') {
      console.error(consoleMsg)
    }
  }
  if (guiConsole) {
    // false means don't append (again) to the dev console
    appendToConsole(consoleMsg, false)
  }
  // log messages go to log file: currently rosout.log
  publishRosLog(logLevel, stamps[0], message)
}

// these functions copy the rospy logging functions
/**
 * Sends a debug message with timestamp to the GUI and chrome consoles.
 * Also publishes it to /rosout.
 * @param message the log message
*/
function logDebug (message, devConsole = true, guiConsole = true) {
  // unlike rospy, (currently) the debug messages we generate will get published
  rosLog(ROSDEBUG, message, devConsole, guiConsole)
}
/**
 * Sends an info message with timestamp to the GUI and chrome consoles.
 * Also publishes it to /rosout.
 * @param message the log message
*/
function logInfo (message, devConsole = true, guiConsole = true) {
  rosLog(ROSINFO, message, devConsole, guiConsole)
}
/**
 * Sends a warning message with timestamp to the GUI and chrome consoles.
 * Also publishes it to /rosout.
 * @param message the log message
*/
function logWarn (message, devConsole = true, guiConsole = true) {
  rosLog(ROSWARN, message, devConsole, guiConsole)
}
/**
 * Sends an error message with timestamp to the GUI and chrome consoles.
 * Also publishes it to /rosout.
 * @param message the log message
*/
function logErr (message, devConsole = true, guiConsole = true) {
  rosLog(ROSERROR, message, devConsole, guiConsole)
}
/**
 * Sends a fatal error message with timestamp to the GUI and chrome consoles.
 * Also publishes it to /rosout.
 * @param message the log message
*/
function logFatal (message, devConsole = true, guiConsole = true) {
  rosLog(ROSFATAL, message, devConsole, guiConsole)
}

function requestMuxChannel (elemID, callback, timeout = REQUEST_TIMEOUT) {
  let dev = elemID[elemID.length - 1]
  let request = new ROSLIB.ServiceRequest({ device: dev })
  let sentTime = new Date().getTime()

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

  logInfo('Sending request to execute command "' + command + '"')

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
ARM_LISTENER_TASK = 'arm_listener'
ROVER_LISTENER_TASK = 'rover_listener'
SCIENCE_LISTENER_TASK = 'science_listener'
PDS_LISTENER_TASK = 'pds_listener'
CAMERA_TASK = 'camera_stream'
CAMERA_PORTS = 'camera_ports'

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
  callback = arg => {},
  reqArgs = '',
  timeout = REQUEST_TIMEOUT
) {

  let request = new ROSLIB.ServiceRequest({ task: reqTask, status: reqStatus, args: reqArgs })
  let sentTime = new Date().getTime()

  if (reqStatus == STATUS_STOP) {
    logInfo('Sending request to stop ' + reqTask + ' task')
  } else if (reqStatus == STATUS_START) {
    logInfo('Sending request to start ' + reqTask + ' task')
  } else if (reqStatus == STATUS_CHECK) {
    logInfo('Sending request to check ' + reqTask + ' task status')
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
    requestTask(ARM_LISTENER_TASK, STATUS_CHECK, (msgs) => {
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
    requestTask(SCIENCE_LISTENER_TASK, STATUS_CHECK, function (msgs) {
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
    requestTask(PDS_LISTENER_TASK, STATUS_CHECK, function (msgs) {
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

function sendRequest (device, command, callback, timeout = REQUEST_TIMEOUT) {
  let request = new ROSLIB.ServiceRequest({ msg: command })
  let sentTime = new Date().getTime()

  logInfo('Sending request to execute command "' + command + '"')

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

/*
returns the IP portion of the currently set ROS_MASTER_URI
*/
function getRoverIP (callback) {
  logInfo('roverIP: ' + env.ROS_MASTER_IP)
  logInfo('hostIP: ' + env.HOST_IP)
  return env.ROS_MASTER_IP
}

/*
command sending
*/
function sendIKCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  logInfo('Sending "' + cmd + '" to IK node')
  ik_command_publisher.publish(cmd)
}

function sendArmCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  logInfo('Sending "' + cmd + '" to arm Teensy')
  arm_command_publisher.publish(command)
}

function sendRoverCommand (cmd) {
  logDebug('Sending "' + cmd + '" to Rover Teensy')
  let command = new ROSLIB.Message({ data: cmd })
  logInfo('Sending "' + cmd + '" to rover Teensy')
  rover_command_publisher.publish(command)
}

function sendPdsCommand (cmd) {
  logDebug('Sending "' + cmd + '" to PDS Teensy')
  let command = new ROSLIB.Message({ data: cmd })
  logInfo('Sending "' + cmd + '" to PDS')
  pds_command_publisher.publish(command)
}

function sendScienceCommand (cmd) {
  let command = new ROSLIB.Message({ data: cmd })
  logInfo('Sending "' + cmd + '" to science Teensy')
  science_command_publisher.publish(command)
}
