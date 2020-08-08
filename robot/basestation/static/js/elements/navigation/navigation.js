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
  }

  function setupAntennaStats(latitude, longitude, heading) {
      $('#antenna-stats-latitude').text(latitude.toFixed(6))
      $('#antenna-stats-longitude').text(longitude.toFixed(6))
      $('#antenna-stats-heading').text(heading + '°')
  }

  function setupGoalStats(latitude, longitude) {
      $('#goal-stats-latitude').text(latitude.toFixed(6))
      $('#goal-stats-longitude').text(longitude.toFixed(6))
  }

  goalList = []
  goalCount = 0
  createAntennaInputButtonsHandler("button[id^='antenna-']")

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
        switch ($("#antenna-latitude-fieldset").attr('format')) {
          case 'decimal-degrees': {
            latitude = parseFloat(($("#antenna-decimal-degrees-decimal-degree-input.latitude").val()))

            if (isNaN(latitude)) {
              throw "Bad Input latitude for decimal-degrees format"
            }

            $("#antenna-decimal-degrees-decimal-degree-input.latitude").attr("value", latitude)
            break;
          }

          case 'degrees-decimal-minutes': {
            let decimal = parseFloat($('#antenna-degrees-decimal-minutes-degree-input.latitude').val())
            let minute = parseFloat($('#antenna-degrees-decimal-minutes-decimal-minute-input.latitude').val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input latitude for degrees-decimal-minutes format"
            }

            $("#antenna-degrees-decimal-minutes-degree-input.latitude").attr("value", decimal)
            $("#antenna-degrees-decimal-minutes-decimal-minute-input.latitude").attr("value", minute)

            latitude = decimal + (minute / 60)
            break;
          }

          case 'degrees-minutes-seconds': {
            let decimal = parseFloat($('#antenna-degrees-minutes-seconds-degree-input.latitude').val())
            let minutes = parseFloat($('#antenna-degrees-minutes-seconds-minute-input.latitude').val())
            let seconds = parseFloat($('#antenna-degrees-minutes-seconds-second-input.latitude').val())

            if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
              throw "Bad input latitude degrees-minutes-seconds format"
            }

            $("#antenna-degrees-minutes-seconds-degree-input.latitude").attr("value", decimal)
            $("#antenna-degrees-minutes-seconds-minute-input.latitude").attr("value", minutes)
            $("#antenna-degrees-minutes-seconds-second-input.latitude").attr("value", seconds)

            latitude = decimal + (minutes / 60 + seconds / 3600)
            break;
          }

          default:
            throw "You didn't enter anything for latitude"
        }

        switch ($("#antenna-longitude-fieldset").attr('format')) {
          case 'decimal-degrees': {
            longitude = parseFloat(($("#antenna-decimal-degrees-decimal-degree-input.longitude").val()))

            if (isNaN(longitude)) {
              throw "Bad Input longitude for decimal-degrees"
            }

            $("#antenna-decimal-degrees-decimal-degree-input.longitude").attr("value", longitude)
            break;
          }

          case 'degrees-decimal-minutes': {
            let decimal = parseFloat($('#antenna-degrees-decimal-minutes-degree-input.longitude').val())
            let minute = parseFloat($('#antenna-degrees-decimal-minutes-decimal-minute-input.longitude').val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input longitude for degrees-decimal-minutes"
            }

            $("#antenna-degrees-decimal-minutes-degree-input.longitude").attr("value", decimal)
            $("#antenna-degrees-decimal-minutes-decimal-minute-input.longitude").attr("value", minute)

            longitude = decimal + (minute / 60)
            break;
          }

          case 'degrees-minutes-seconds': {
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
            break;
          }

          default:
            throw "You didn't enter anything for longitude"
        }

        let bearing = parseFloat($("#antenna-bearing-input").val())
        if (isNaN(bearing)) {
          throw "Bad input bearing"
        }

        $("#antenna-bearing-input").attr("value", bearing)


        $('.antenna-input-field').prop('disabled', true)
        $('.antenna-change-btn').prop('disabled', false)
        $('#antenna-confirm-btn').prop('disabled', true)

        //ROS params
        antenna_latitude.set(latitude)
        antenna_longitude.set(longitude)
        antenna_start_dir.set(bearing)
        setupAntennaStats(latitude, longitude, bearing)
        logInfo('Antenna parameters have been set!')
    } catch (e) {
        logErr(e)
    }
  }

  createGoalsInputButtonsHandler("button[id^='goal-']")

  // set goals info
  // onClick events for goals data entry buttons
  function createGoalsInputButtonsHandler(button) {
    $(button).mouseup(e => {
      event.preventDefault()
      let buttonId = $(e.target).attr("id").split("-")
      let buttonClass = $(e.target).attr("class").split(" ")
      if ((buttonId[1] == 'latitude' || buttonId[1] == 'longitude') && (buttonId[3] != "format")) {
        // select latitude or longitude input format
        let mode = buttonId[1]
        let format = buttonId[2] + "-" + buttonId[3]
        let goalNum = buttonClass[3].replace('goal-', '')

        if (buttonId[4] != 'btn') {
          format = format + "-" + buttonId[4]
        }

        goalHandlerTemplate(mode, format, goalNum)

      } else {
          switch (buttonId[1]) {
            case 'toggle': {
              // add new and its input templates
              toggleGoals()
              break;
            }

            case 'new': {
              // add new and its input templates
              addGoal()
              break;
            }

            case 'confirm': {
              // confirm and set goal info
              let goalNum = buttonClass[2].replace('goal-', '')
              goalConfirmButtonHandler(goalNum)
              break;
            }

            case 'delete': {
              // delete goal
              let goalNum = buttonClass[2].replace('goal-', '')
              goalDeleteButtonHandler(goalNum)
              break;
            }
          }
        }
    })
  }

  // functions
  function toggleGoals() {
    let goalTemplate = $('#created-goal-template').html()
    for (let i = 0; i < goalCount; i++) {
      $("#goal-modal-body-content").append(goalTemplate)
      $("#goal-modal-body-content .goal").addClass('goal-' + i).removeClass('goal')
      $(".goal-" + i).find('*').addClass('goal-' + i)

      createGoalButtons(i)

      $('#goal-name.goal-' + i).val(goalList[i].name)
      $('#goal-confirm-btn.goal-' + i).prop('disabled', true)
      $('#goal-change-btn.goal-' + i).prop('disabled', false)
      $('div.goal-' + i + ' fieldset').prop('disabled', true)
      $('#goal-decimal-degrees-decimal-degree-input.latitude.goal-' + i).val(goalList[i].latitude)
      $('#goal-decimal-degrees-decimal-degree-input.longitude.goal-' + i).val(goalList[i].longitude)
    }
  }

  function addGoal() {
    let newGoalTemplate = $('#add-goal-coordinates').html()

    $("#goal-modal-body-content").append(newGoalTemplate)
    $("#goal-modal-body-content .goal").addClass('goal-' + goalCount).removeClass('goal')
    $(".goal-" + goalCount).find('*').addClass('goal-' + goalCount)

    createGoalButtons(goalCount)
    goalCount++
  }

  function goalHandlerTemplate(mode, format, current) {
    $('#goal-' + mode + '-fieldset.goal-' + current).attr("format", format)
    $('#goal-' + mode + '-select-format.goal-' + current).dropdown('toggle')
    $('#goal-' + mode + '-select-format.goal-' + current).detach()

    let inputTemplate = $('#goal-' + format + '-input-template').html()
    $('#goal-' + mode + '-input-group.goal-' + current).append(inputTemplate)
    $('#goal-' + mode + '-input-group.goal-' + current).find('input').addClass('goal-' + current + ' ' + mode)
    $('#goal-' + mode + '-input-group.goal-' + current + ' span.input-group-text:first').text(mode)
  }

  function createGoalButtons(current) {
    let goalButtons = $("#goal-buttons").html()
    let currentGoalButton = '#goal-buttons-input-group.goal-' + current
    $(currentGoalButton).append(goalButtons)
    $(currentGoalButton).find('*').addClass('goal-' + current)

    createGoalsInputButtonsHandler("button[id^='goal-'].goal-" + current)
  }

  function goalConfirmButtonHandler(current) {
        let latitude_format = null
        let longitude_format = null

        latitude_format = $('#goal-latitude-fieldset.goal-' + current).attr('format')
        longitude_format = $('#goal-longitude-fieldset.goal-' + current).attr('format')

        $('#goal-change-btn.goal-' + current).prop('disabled', false)
        $('#goal-confirm-btn.goal-' + current).prop('disabled', true)

        $('#goal-latitude-select-format.goal-' + current).detach()
        $('#goal-longitude-select-format.goal-' + current).detach()

        setGoalData(current, latitude_format, longitude_format)
  }

  function goalDeleteButtonHandler(current) {
    /*------------------------\
    implement ROS goal deleting
    \------------------------*/
    let goalName = $('#goal-name.goal-' + current).val()
    let msg = new ROSLIB.Message({
        data: goalName
    })
    delete_goal_publisher.publish(msg)

    $('.goal-' + current).remove()
    goalCount--
  }

  function setGoalData(current, latitude_format, longitude_format) {
    let latitude = null
    let longitude = null

    let goalElem = $('#goal-name.goal-' + current)
    let goalName = goalElem.val()

    try {
        for (let i = 0; i < goalList.length; i++) {
            if (goalName == goalList[i].name) {
                throw 'Enter different name'
            }
        }

        switch (latitude_format) {
          case 'decimal-degrees': {
            latitude = parseFloat($('#goal-decimal-degrees-decimal-degree-input.latitude.goal-' + current).val())

            if (isNaN(latitude)) {
              throw "Bad input latitude for decimal degrees format"
            }

            $('#goal-decimal-degrees-decimal-degree-input.latitude.goal-' + current).attr("value", latitude)
            break;
          }

          case 'degrees-decimal-minutes': {
            let decimal = parseFloat($('#goal-degrees-decimal-minutes-degree-input.latitude.goal-' + current).val())
            let minute = parseFloat($('#goal-degrees-decimal-minutes-decimal-minute-input.latitude.goal-' + current).val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input latitude for degrees-decimal-minutes format"
            }

            $('#goal-degrees-decimal-minutes-degree-input.latitude.goal-' + current).attr("value", decimal)
            $('#goal-degrees-decimal-minutes-decimal-minute-input.latitude.goal-' + current).attr("value", minute)

            latitude = decimal + (minute / 60)
            break;
          }

          case 'degrees-minutes-seconds': {
            let decimal = parseFloat($('#goal-degrees-minutes-seconds-degree-input.latitude.goal-' + current).val())
            let minutes = parseFloat($('#goal-degrees-minutes-seconds-minute-input.latitude.goal-' + current).val())
            let seconds = parseFloat($('#goal-degrees-minutes-seconds-second-input.latitude.goal-' + current).val())

            if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
              throw "Bad input latitude for degrees-minutes-seconds format"
            }

            $('#goal-degrees-minutes-seconds-degee-input.latitude.goal-' + current).attr("value", decimal)
            $('#goal-degrees-minutes-seconds-minute-input.latitude.goal-' + current).attr("value", minutes)
            $('#goal-degrees-minutes-seconds-second-input.latitude.goal-' + current).attr("value", seconds)

            latitude = decimal + (minutes / 60 + seconds / 3600)
            break;
          }

          default:
            throw "You didn't enter anything for latitude"
        }

        switch (longitude_format) {
          case 'decimal-degrees': {
            longitude = parseFloat($('#goal-decimal-degrees-decimal-degree-input.longitude.goal-' + current).val())

            if (isNaN(longitude)) {
              throw "Bad input longitude for decimal-degrees format"
            }

            $('#goal-decimal-degrees-decimal-degree-input.longitude.goal-' + current).attr("value", longitude)
            break;
          }

          case 'degrees-decimal-minutes': {
            let decimal = parseFloat($('#goal-degrees-decimal-minutes-degree-input.longitude.goal-' + current).val())
            let minute = parseFloat($('#goal-degrees-decimal-minutes-decimal-minute-input.longitude.goal-' + current).val())

            if (isNaN(decimal) || isNaN(minute)) {
              throw "Bad input longitude for degrees-decimal-minutes format"
            }

            $('#goal-degrees-decimal-minutes-decimal-input.longitude.goal-' + current).attr("value", decimal)
            $('#goal-degrees-decimal-minutes-decimal-minute-input.longitude.goal-' + current).attr("value", minute)

            longitude = decimal + (minute / 60)
            break;
          }

          case 'degrees-minutes-seconds': {
            let decimal = parseFloat($('#goal-degrees-minutes-seconds-degree-input.longitude.goal-' + current).val())
            let minutes = parseFloat($('#goal-degrees-minutes-seconds-minute-input.longitude.goal-' + current).val())
            let seconds = parseFloat($('#goal-degrees-minutes-seconds-second-input.longitude.goal-' + current).val())

            if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
              throw "Bad input long for degrees-minutes-seconds format"
            }

            $('#goal-degrees-minutes-seconds-degree-input.longitude.goal-' + current).attr("value", decimal)
            $('#goal-degrees-minutes-seconds-minute-input.longitude.goal-' + current).attr("value", minutes)
            $('#goal-degrees-minutes-seconds-second-input.longitude.goal-' + current).attr("value", seconds)

            longitude = decimal + (minutes / 60 + seconds / 3600)
            break;
          }

          default:
            throw "You didn't enter anything for longitude"
        }

        if (goalName.length != 0) {
          goalElem.attr('value', goalName)
        } else {
          goalElem.attr('value', 'Goal-' + current)
        }

        $('#goal-name-fieldset.goal-' + current).prop('disabled', true)
        $('#goal-latitude-fieldset.goal-' + current).prop('disabled', true)
        $('#goal-longitude-fieldset.goal-' + current).prop('disabled', true)


        /*----------------------------------\
        this section implements ROS goal node
        \----------------------------------*/
        let goalData = new ROSLIB.Message({
            name: goalName,
            longitude: longitude,
            latitude: latitude
        })

        goalList.push(goalData)

        let goalDataList = new ROSLIB.Message({
            goal_list: goalList
        })

        goal_list_subscriber.publish(goalDataList)

        logInfo('Goal parameters have been set!')
      } catch (e) {
        $('#goal-confirm-btn.goal-' + current).prop('disabled', false)
        $('#goal-change-btn.goal-' + current).prop('disabled', true)

        logWarn(e)
    }
  }


  function updateAntennaParams() {
      antenna_latitude.get(function(latitude) {
          antenna_longitude.get(function(longitude) {
              antenna_start_dir.get(function(heading) {
                  if (latitude && longitude && heading) {
                      setupAntennaStats(latitude, longitude, heading)
                  }
              })
          })
      })
  }
})
