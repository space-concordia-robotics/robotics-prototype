$(document).ready(() => {
  // setup a subscriber for the rover_position topic
  let rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'mcu_control/RoverPosition'
  })
  rover_position_listener.subscribe(function (message) {
    $('#rover-latitude').text(message.latitude)
    $('#rover-longitude').text(message.longitude)
    $('#rover-heading').text(message.heading)
  })
  // setup a subscriber for the antenna_goal topic
  let antenna_goal_listener = new ROSLIB.Topic({
    name: 'antenna_goal',
    messageType: 'mcu_control/AntennaGoal',
    ros: ros
  })
  antenna_goal_listener.subscribe(function (message) {
    $('#recommended-antenna-angle').text(parseFloat(message.desiredDir).toFixed(3))
    $('#distance-to-rover').text(parseFloat(message.distFromBase).toFixed(2))
  })
  // setup gps parameters for antenna directing
  let antenna_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_latitude'
  })
  let antenna_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_longitude'
  })
  let antenna_start_dir = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_start_dir'
  })

  // setup a subscriber for the rover_goal topic
  let rover_goal_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_goal',
    messageType: 'mcu_control/rover_goal'
  })
  rover_goal_listener.subscribe(function (message) {
    $('#recommended-rover-heading').text(parseFloat(message.desiredDir).toFixed(3))
    $('#distance-to-goal').text(parseFloat(message.distToGoal).toFixed(2))
  })
  // setup gps parameters for rover goals
  let goal_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_latitude'
  })
  let goal_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_longitude'
  })

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

  $('#send-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#antenna-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-start-dir-input').val()) {
      appendToConsole('bearing field empty!')
      goodInput = false
    }
    if (goodInput) {
      let initialLatitude = $('#antenna-latitude-input').val()
      let initialLongitude = $('#antenna-longitude-input').val()
      let initialBearing = $('#antenna-start-dir-input').val()
      antenna_latitude.set(parseFloat(initialLatitude))
      antenna_longitude.set(parseFloat(initialLongitude))
      antenna_start_dir.set(parseFloat(initialBearing))
      $('#antenna-latitude').text(initialLatitude)
      $('#antenna-longitude').text(initialLongitude)
      $('#antenna-start-dir').text(initialBearing)
      $('#antenna-inputs').hide()
      $('#antenna-unchanging').show()
    }
  })
  $('#change-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    $('#antenna-inputs').show()
    $('#antenna-unchanging').hide()
  })

  $('#send-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#goal-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#goal-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (goodInput) {
      let desiredLatitude = $('#goal-latitude-input').val()
      let desiredLongitude = $('#goal-longitude-input').val()
      goal_latitude.set(parseFloat(desiredLatitude))
      goal_longitude.set(parseFloat(desiredLongitude))
      $('#goal-latitude').text(desiredLatitude)
      $('#goal-longitude').text(desiredLongitude)
      $('#goal-inputs').hide()
      $('#goal-unchanging').show()
    }
  })

  $('#change-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    $('#goal-inputs').show()
    $('#goal-unchanging').hide()
  })
})
