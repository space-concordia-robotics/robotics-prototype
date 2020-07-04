$(document).ready(() => {
  // setup a subscriber for the rover_position topic
  let rover_position_listener = new ROSLIB.Topic({
      ros: ros,
      name: 'rover_position',
      messageType: 'geometry_msgs/Point'
  })
  rover_position_listener.subscribe(function(message) {
      $('#rover-latitude').text(message.x)
      $('#rover-longitude').text(message.y)
      $('#rover-heading').text(message.z)
  })
  // setup a subscriber for the antenna_goal topic
  let antenna_goal_listener = new ROSLIB.Topic({
      name: 'antenna_goal',
      messageType: 'geometry_msgs/Point',
      ros: ros
  })
  antenna_goal_listener.subscribe(function(message) {
      $('#antenna-stats-rec-angle').text(parseFloat(message.x).toFixed(3))
      $('#antenna-stats-dist-to-rover').text(parseFloat(message.y).toFixed(2))
  })
  // setup topics to communicate with GoalsNode
  let create_goal_publisher = new ROSLIB.Topic({
      ros: ros,
      name: 'create_goal',
      messageType: 'mcu_control/RoverGoal'
  })
  let delete_goal_publisher = new ROSLIB.Topic({
      ros: ros,
      name: 'delete_goal',
      messageType: 'std_msgs/String'
  })
  goal_list_subscriber.subscribe(function(message) {
      goalList = message.goal_list
      if (goalCount != goalList.length) {
          goalCount = goalList.length

      }
      if (goalCount > 0) {
          goal_latitude.set(goalList[0].latitude)
          goal_longitude.set(goalList[0].longitude)
          $('#goal-stats-latitude').text(goalList[0].latitude.toFixed(6))
          $('#goal-stats-longitude').text(goalList[0].longitude.toFixed(6))
      } else {
          goal_latitude.set(false)
          goal_longitude.set(false)
          $('#goal-stats-latitude').text('----')
          $('#goal-stats-longitude').text('----')
      }

      // this is a meme implementation of a loop to update the params without resorting to a message type + subscriber + publisher
      updateAntennaParams()
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
  let has_gps_goal = new ROSLIB.Param({
      ros: ros,
      name: 'has_gps_goal'
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
  // setup a subscriber for the rover_goal topic
  new ROSLIB.Topic({
      ros: ros,
      name: 'rover_goal',
      messageType: 'geometry_msgs/Point'
  });

  initNavigationPanel()
  createAntennaInputButtonsHandler("button[id^='antenna-']")

  function initNavigationPanel() {
      antenna_latitude.get(function(latitude) {
          antenna_longitude.get(function(longitude) {
              antenna_start_dir.get(function(heading) {
                  if (latitude && longitude && heading) {
                      logInfo('Found antenna parameters to set')
                      setupAntennaStats(latitude, longitude, heading)
                  } else {
                      logInfo('Antenna parameters were not found')
                  }
              })
          })
      })

      goal_latitude.get(function(latitude) {
          goal_longitude.get(function(longitude) {
              if (latitude && longitude) {
                  has_gps_goal.set(true)
                  logInfo('Found goal coordinates to set')
                  setupGoalStats(latitude, longitude)
              } else {
                  logInfo('Goal parameters not found')
              }
          })
      })

      function setupAntennaStats(latitude, longitude, heading) {
          $('#antenna-stats-latitude').text(latitude.toFixed(6))
          $('#antenna-stats-longitude').text(longitude.toFixed(6))
          $('#antenna-stats-heading').text(heading + '°')
      }

      function setupGoalStats(latitude, longitude) {
          $('#goal-stats-latitude').text(latitude.toFixed(6))
          $('#goal-stats-longitude').text(longitude.toFixed(6))
      }
  }

  goalList = {}

  // set antenna data
  // onClick events for antenna data entry buttons
  function createAntennaInputButtonsHandler(button, detachedData) {
    $(button).mouseup(e => {
      event.preventDefault()
      let buttonId = $(e.target).attr("id").split("-")
      let buttonClass = $(e.target).attr("class").split(" ")
      if (buttonId[1] == 'latitude' || buttonId[1] == 'longitude') {
        // select latitude or longitude input format
        let mode = buttonId[1]
        let format = buttonId[2] + "-" + buttonId[3]
        if (buttonId[4] != 'btn') {
          format = format + "-" + buttonId[4]
        }
        antennaHandlerTemplate(mode, format)
      } else if (buttonClass[2] == 'antenna-change-btn') {
          // change latitude or longitude and its format
          if (buttonId[1] != 'bearing') {
            let mode = buttonClass[4]
            let format = buttonId[1] + "-" + buttonId[2]
            if (buttonId[3] != 'change') {
              format = format + "-" + buttonId[3]
            }
            antennaChangeButtonHandlerTemplate(mode, format, detachedData)
          } else {
              // change bearing
              antennaBearingChange()
          }
      } else if (buttonId[1] == "confirm") {
          // confirm, set and save info
          setAntennaData()
      }
    })
  }

  function antennaHandlerTemplate(mode, format) {
    $('#antenna-' + mode + '-fieldset').attr('format', format)
    $('#antenna-select-' + mode + '-format-btn').dropdown('toggle')
    let detachedData = $('#antenna-select-' + mode + '-format').detach()
    $("#antenna-" + mode + "-fieldset").empty()

    let antennaInputTemplate = $("div." + format + "-template").html()
    $("#antenna-" + mode + "-fieldset").append(antennaInputTemplate).find('*').addClass(format + " " + mode)
    $("#antenna-" + mode + "-fieldset span." + format + ":first").text(mode)
    $('.antenna-input-field.' + mode).prop('disabled', false)

    createAntennaInputButtonsHandler('#antenna-' + format + '-change-btn.' + mode, detachedData)
  }

  function antennaChangeButtonHandlerTemplate(mode, format, detachedData) {
    $("#antenna-" + mode + "-input-group").empty()
    $("#antenna-" + mode + "-fieldset").append(detachedData)
    $("#antenna-confirm-btn").prop('disabled', false)
  }

  function createAntennaBearingChangeButtonHandler() {
    $("#antenna-bearing-change-btn").on('click', function(event) {
      $("#antenna-bearing-input").prop('disabled', false)
      $("#antenna-bearing-change-btn").prop('disabled', false)
      $("#antenna-confirm-btn").prop('disabled', false)
    })
  }

  function antennaBearingChange() {
    $("#antenna-bearing-input").prop('disabled', false)
    $("#antenna-bearing-change-btn").prop('disabled', false)
    $("#antenna-confirm-btn").prop('disabled', false)
  }

  function setAntennaData() {
    let latitude = null
    let longitude = null

    try {
        if ($("#antenna-latitude-fieldset").attr('format') == 'decimal-degrees') {
            let decimaldegree = parseFloat(($("#antenna-decimal-degrees-decimal-degree-input.latitude").val()))
            $("#antenna-decimal-degrees-decimal-degree-input.latitude").attr("value", decimaldegree)

            if (isNaN(decimaldegree)) {
              throw "Bad Input latitude for decimal-degrees"
            }

            latitude = decimaldegree
        } else if ($("#antenna-latitude-fieldset").attr('format') == 'degrees-decimal-minutes') {
            let decimal = parseFloat($('#antenna-degrees-decimal-minutes-degree-input.latitude').val())
            let minute = parseFloat($('#antenna-degrees-decimal-minutes-decimal-minute-input.latitude').val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input latitude for degrees-decimal-minutes"
            }

            $("#antenna-degrees-decimal-minutes-degree-input.latitude").attr("value", decimal)
            $("#antenna-degrees-decimal-minutes-decimal-minute-input.latitude").attr("value", minute)

            latitude = decimal + (minute / 60)
        } else if ($("#antenna-latitude-fieldset").attr('format') == 'degrees-minutes-seconds') {
            let decimal = parseFloat($('#antenna-degrees-minutes-seconds-deg-input.latitude').val())
            let minutes = parseFloat($('#antenna-degrees-minutes-seconds-minute-input.latitude').val())
            let seconds = parseFloat($('#antenna-degrees-minutes-seconds-second-input.latitude').val())

            if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
              throw "Bad input latitude degrees-minutes-seconds"
            }

            $("#antenna-degrees-minutes-seconds-degree-input.latitude").attr("value", decimal)
            $("#antenna-degrees-minutes-seconds-minute-input.latitude").attr("value", minutes)
            $("#antenna-degrees-minutes-seconds-second-input.latitude").attr("value", seconds)

            latitude = decimal + (minutes / 60 + seconds / 3600)
        }

        if ($("#antenna-longitude-fieldset").attr('format') == 'decimal-degrees') {
            let decimaldegree = parseFloat(($("#antenna-decimal-degrees-decimal-degree-input.longitude").val()))

            if (isNaN(decimaldegree)) {
              throw "Bad Input longitude for decimal-degrees"
            }

            $("#antenna-decimal-degrees-decimal-degree-input.longitude").attr("value", decimaldegree)

            longitude = decimaldegree
        } else if ($("#antenna-longitude-fieldset").attr('format') == 'degrees-decimal-minutes') {
            let decimal = parseFloat($('#antenna-degrees-decimal-minutes-degree-input.longitude').val())
            let minute = parseFloat($('#antenna-degrees-decimal-minutes-decimal-minute-input.longitude').val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input longitude for degrees-decimal-minutes"
            }

            $("#antenna-degrees-decimal-minutes-degree-input.longitude").attr("value", decimal)
            $("#antenna-degrees-decimal-minutes-decimal-minute-input.longitude").attr("value", minute)

            longitude = decimal + (minute / 60)
        } else if ($("#antenna-longitude-fieldset").attr('format') == 'degrees-minutes-seconds') {
            let decimal = parseFloat($('#antenna-degrees-minutes-seconds-degree-input.longitude').val())
            let minutes = parseFloat($('#antenna-degrees-minutes-seconds-minute-input.longitude').val())
            let seconds = parseFloat($('#antenna-degrees-minutes-seconds-second-input.longitude').val())

            if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
              throw "Bad input longitude for degrees-minutes-seconds"
            }

            $("#antenna-degrees-minutes-seconds-degree-input.longitude").attr("value", decimal)
            $("#antenna-degrees-minutes-seconds-minute-input.longitude").attr("value", minutes)
            $("#antenna-degrees-minutes-seconds-second-input.longitude").attr("value", seconds)

            longitude = decimal + (minutes / 60 + seconds / 3600)
        }
        let bearing = parseFloat($("#antenna-bearing-input").val())
        if (isNaN(bearing)) {
          throw "Bad input bearing"
        }

        $("#antenna-bearing-input").attr("value", bearing)


        $('.antenna-input-field').prop('disabled', true)
        $('.antenna-change-btn').prop('disabled', false)
        $('#antenna-confirm-btn').prop('disabled', true)

        //createAntennaInputButtonsHandler('#antenna-bearing-change-btn')

        //ROS params
        antenna_latitude.set(latitude)
        antenna_longitude.set(longitude)
        antenna_start_dir.set(bearing)
        $('#antenna-stats-latitude').text(latitude.toFixed(6))
        $('#antenna-stats-longitude').text(longitude.toFixed(6))
        $('#antenna-stats-heading').text(bearing)
        logInfo('Antenna parameters have been set!')
    } catch (e) {
        logErr(e)
    }
  }

  // set goals info
  // onClick events for goals data entry buttons
  function createGoalsInputButtonsHandler(button, detachedData) {
    $(button).mouseup(e => {
      console.log('TEST')
      event.preventDefault()
      let buttonId = $(e.target).attr("id").split("-")
      let buttonClass = $(e.target).attr("class").split(" ")
      if (buttonId[1] == 'latitude' || buttonId[1] == 'longitude') {
        // select latitude or longitude input format
        let mode = buttonId[1]
        let format = buttonId[2] + "-" + buttonId[3]
        if (buttonId[4] != 'btn') {
          format = format + "-" + buttonId[4]
        }
        antennaHandlerTemplate(mode, format)
      } else if (buttonClass[2] == 'antenna-change-btn') {
          // change latitude or longitude and its format
          if (buttonId[1] != 'bearing') {
            let mode = buttonClass[4]
            let format = buttonId[1] + "-" + buttonId[2]
            if (buttonId[3] != 'change') {
              format = format + "-" + buttonId[3]
            }
            antennaChangeButtonHandlerTemplate(mode, format, detachedData)
          } else {
              // change bearing
              antennaBearingChange()
          }
      } else if (buttonId[1] == "confirm") {
          // confirm, set and save info
          setAntennaData()
      }
    })
  }

})
