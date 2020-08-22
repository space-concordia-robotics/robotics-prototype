$(document).ready(() => {
  goalList = []
  goalCount = 0

  const minuteTodecmial = 60
  const secondTodecmial = 3600

  // setup a subscriber for the rover_position topic
  const rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'geometry_msgs/Point'
  })
  rover_position_listener.subscribe(function (message) {
    $('#rover-latitude').text(message.x)
    $('#rover-longitude').text(message.y)
    $('#rover-heading').text(message.z)
  })
  // setup a subscriber for the antenna_goal topic
  const antenna_goal_listener = new ROSLIB.Topic({
    name: 'antenna_goal',
    messageType: 'geometry_msgs/Point',
    ros: ros
  })
  // setup topics to communicate with GoalsNode
  const create_goal_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'create_goal',
    messageType: 'mcu_control/RoverGoal'
  })
  const delete_goal_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'delete_goal',
    messageType: 'std_msgs/String'
  })

  goal_list_subscriber.subscribe(function (message) {
    if (message.goal_list.length > 0) {
      if (goalCount == 0) {
        goalCount = message.goal_list.length
      }
      if (goalList.length != message.goal_list.length) {
        goalList = message.goal_list
        toggleGoals()
      }
      goalList = message.goal_list
      $('#goal-stats-latitude').text(goalList[0].latitude.toFixed(6))
      $('#goal-stats-longitude').text(goalList[0].longitude.toFixed(6))
    } else {
      $('#goal-stats-latitude').text('----')
      $('#goal-stats-longitude').text('----')
    }

    updateAntennaParams()
  })

  // setup gps parameters for antenna directing
  const antenna_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_latitude'
  })
  const antenna_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_longitude'
  })
  const antenna_start_dir = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_start_dir'
  })
  const has_gps_goal = new ROSLIB.Param({
    ros: ros,
    name: 'has_gps_goal'
  })
  // setup a subscriber for the rover_goal topic
  new ROSLIB.Topic({
    ros: ros,
    name: 'rover_goal',
    messageType: 'geometry_msgs/Point'
  })

  function setupAntennaStats (latitude, longitude, heading) {
    $('#antenna-stats-latitude').text(latitude.toFixed(6))
    $('#antenna-stats-longitude').text(longitude.toFixed(6))
    $('#antenna-stats-heading').text(heading + '°')
  }

  function setupGoalStats (latitude, longitude) {
    $('#goal-stats-latitude').text(latitude.toFixed(6))
    $('#goal-stats-longitude').text(longitude.toFixed(6))
  }

  // set antenna data
  // onClick events for antenna data entry buttons
  createAntennaFormatButtonsHandler()
  createBearingChangeButtonHandler()
  createSetAntennaDataButtonsHandler()

  function createAntennaFormatButtonsHandler () {
      // select latitude or longitude input format buttons
      $("button[id^='antenna-latitude'], button[id^='antenna-longitude'").mouseup(e => {
          const buttonId = $(e.target).attr('id').split('-')
          const mode = buttonId[1]
          let format = buttonId[2] + '-' + buttonId[3]
          if (buttonId[4] != 'btn') {
            format = format + '-' + buttonId[4]
          }
          antennaHandlerTemplate(mode, format)
      })
  }

  function createAntennaChangeButtonsHandler (button, mode, detachedData) {
      // unhide antenna format selection box when change button is clicked
      $(button).mouseup(e => {
          antennaChangeButtonHandlerTemplate(mode, detachedData)
      })
  }

  function createBearingChangeButtonHandler () {
      // allow changing of antenna bearing
      $('#antenna-bearing-change-btn').mouseup(e => {
          antennaBearingChange()
      })
  }

  function createSetAntennaDataButtonsHandler () {
      $("button[id='antenna-confirm-btn']").mouseup(e => {
          setAntennaData()
      })
  }

  // AntennaData functions
  function antennaHandlerTemplate (mode, format) {
    $('#antenna-' + mode + '-fieldset').attr('format', format)
    $('#antenna-select-' + mode + '-format-btn').dropdown('toggle')
    const detachedData = $('#antenna-select-' + mode + '-format').detach()
    $('#antenna-' + mode + '-fieldset').empty()

    const antennaInputTemplate = $('div.' + format + '-template').html()
    $('#antenna-' + mode + '-fieldset')
      .append(antennaInputTemplate)
      .find('*')
      .addClass(format + ' ' + mode)
    $('#antenna-' + mode + '-fieldset span.' + format + ':first').text(mode)
    $('.antenna-input-field.' + mode).prop('disabled', false)

    createAntennaChangeButtonsHandler("button[id='antenna-" + format + "-change-btn']", mode, detachedData)
  }

  function antennaChangeButtonHandlerTemplate (mode, detachedData) {
    $('#antenna-' + mode + '-input-group').empty()
    $('#antenna-' + mode + '-fieldset').append(detachedData)
    $('#antenna-confirm-btn').prop('disabled', false)
  }

  function createAntennaBearingChangeButtonHandler () {
    $('#antenna-bearing-change-btn').on('click', function (event) {
      $('#antenna-bearing-input').prop('disabled', false)
      $('#antenna-bearing-change-btn').prop('disabled', false)
      $('#antenna-confirm-btn').prop('disabled', false)
    })
  }

  function antennaBearingChange () {
    $('#antenna-bearing-input').prop('disabled', false)
    $('#antenna-bearing-change-btn').prop('disabled', false)
    $('#antenna-confirm-btn').prop('disabled', false)
  }

  function setAntennaData () {
    let latitude = null
    let longitude = null

    try {
      switch ($('#antenna-latitude-fieldset').attr('format')) {
        case 'decimal-degrees': {
          latitude = parseFloat(
            $('#antenna-decimal-degrees-decimal-degree-input.latitude').val()
          )

          if (isNaN(latitude)) {
            throw 'Bad Input latitude for decimal-degrees format'
          }

          $('#antenna-decimal-degrees-decimal-degree-input.latitude').attr(
            'value',
            latitude
          )
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $('#antenna-degrees-decimal-minutes-degree-input.latitude').val()
          )
          const minute = parseFloat(
            $(
              '#antenna-degrees-decimal-minutes-decimal-minute-input.latitude'
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input latitude for degrees-decimal-minutes format'
          }

          $('#antenna-degrees-decimal-minutes-degree-input.latitude').attr(
            'value',
            decimal
          )
          $(
            '#antenna-degrees-decimal-minutes-decimal-minute-input.latitude'
          ).attr('value', minute)

          latitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $('#antenna-degrees-minutes-seconds-degree-input.latitude').val()
          )
          const minutes = parseFloat(
            $('#antenna-degrees-minutes-seconds-minute-input.latitude').val()
          )
          const seconds = parseFloat(
            $('#antenna-degrees-minutes-seconds-second-input.latitude').val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input latitude degrees-minutes-seconds format'
          }

          $('#antenna-degrees-minutes-seconds-degree-input.latitude').attr(
            'value',
            decimal
          )
          $('#antenna-degrees-minutes-seconds-minute-input.latitude').attr(
            'value',
            minutes
          )
          $('#antenna-degrees-minutes-seconds-second-input.latitude').attr(
            'value',
            seconds
          )

          latitude =
            decimal + (minutes / minuteTodecmial + seconds / secondTodecmial)
          break
        }

        default:
          throw "You didn't enter anything for latitude"
      }

      switch ($('#antenna-longitude-fieldset').attr('format')) {
        case 'decimal-degrees': {
          longitude = parseFloat(
            $('#antenna-decimal-degrees-decimal-degree-input.longitude').val()
          )

          if (isNaN(longitude)) {
            throw 'Bad Input longitude for decimal-degrees'
          }

          $('#antenna-decimal-degrees-decimal-degree-input.longitude').attr(
            'value',
            longitude
          )
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $('#antenna-degrees-decimal-minutes-degree-input.longitude').val()
          )
          const minute = parseFloat(
            $(
              '#antenna-degrees-decimal-minutes-decimal-minute-input.longitude'
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input longitude for degrees-decimal-minutes'
          }

          $('#antenna-degrees-decimal-minutes-degree-input.longitude').attr(
            'value',
            decimal
          )
          $(
            '#antenna-degrees-decimal-minutes-decimal-minute-input.longitude'
          ).attr('value', minute)

          longitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $('#antenna-degrees-minutes-seconds-degree-input.longitude').val()
          )
          const minutes = parseFloat(
            $('#antenna-degrees-minutes-seconds-minute-input.longitude').val()
          )
          const seconds = parseFloat(
            $('#antenna-degrees-minutes-seconds-second-input.longitude').val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input longitude for degrees-minutes-seconds'
          }

          $('#antenna-degrees-minutes-seconds-degree-input.longitude').attr(
            'value',
            decimal
          )
          $('#antenna-degrees-minutes-seconds-minute-input.longitude').attr(
            'value',
            minutes
          )
          $('#antenna-degrees-minutes-seconds-second-input.longitude').attr(
            'value',
            seconds
          )

          longitude =
            decimal + (minutes / minuteTodecmial + seconds / secondTodecmial)
          break
        }

        default:
          throw "You didn't enter anything for longitude"
      }

      const bearing = parseFloat($('#antenna-bearing-input').val())
      if (isNaN(bearing)) {
        throw 'Bad input bearing'
      }

      $('#antenna-bearing-input').attr('value', bearing)

      $('.antenna-input-field').prop('disabled', true)
      $('.antenna-change-btn').prop('disabled', false)
      $('#antenna-confirm-btn').prop('disabled', true)

      // ROS params
      antenna_latitude.set(latitude)
      antenna_longitude.set(longitude)
      antenna_start_dir.set(bearing)
      setupAntennaStats(latitude, longitude, bearing)
      logInfo('Antenna parameters have been set!')
    } catch (e) {
      logErr(e)
    }
  }

  // set goals info
  // mouseup events for goals data entry buttons
  createGoalsAddButtonHandler()

  function createGoalsFormatButtonsHandler (button) {
      // select latitude or longitude input format buttons
      $(button).mouseup(e => {
          const buttonId = $(e.target).attr('id').split('-')
          const buttonClass = $(e.target).attr('class').split(' ')
          const mode = buttonId[1]
          let format = buttonId[2] + '-' + buttonId[3]
          const goalNum = buttonClass[3].replace('goal-', '')

          if (buttonId[4] != 'btn') {
            format = format + '-' + buttonId[4]
          }

          goalHandlerTemplate(mode, format, goalNum)
      })
  }

  function createGoalsAddButtonHandler () {
      // add new goal templates
      $("button[id='goal-new-coordinates-btn']").mouseup(e => {
          addGoal()
      })
  }

  function createGoalsConfirmButtonHandler (goalNum) {
      // add new goal templates
      $('#goal-confirm-btn.goal-' + goalNum).mouseup(e => {
          goalConfirmButtonHandler(goalNum)
      })
  }

  function createGoalsDeleteButtonHandler (goalNum) {
      // add new goal templates
      $('#goal-delete-btn.goal-' + goalNum).mouseup(e => {
          goalDeleteButtonHandler(goalNum)
      })
  }

  // functions
  function toggleGoals() {
    $('div[class*="goal-"]').remove()
    const goalTemplate = $('#created-goal-template').html()
    for (let i = 0; i < goalList.length; i++) {
      $('#goal-modal-body-content').append(goalTemplate)
      $('#goal-modal-body-content .goal')
        .addClass('goal-' + i)
        .removeClass('goal')
      $('.goal-' + i)
        .find('*')
        .addClass('goal-' + i)

      createGoalButtons(i)

      $('#goal-name.goal-' + i).val(goalList[i].name)
      $('#goal-confirm-btn.goal-' + i).prop('disabled', true)
      $('#goal-change-btn.goal-' + i).prop('disabled', false)
      $('div.goal-' + i + ' fieldset').prop('disabled', true)
      $('#goal-decimal-degrees-decimal-degree-input.latitude.goal-' + i).val(goalList[i].latitude)
      $('#goal-decimal-degrees-decimal-degree-input.longitude.goal-' + i).val(goalList[i].longitude)
    }
  }

  function addGoal () {
    const newGoalTemplate = $('#add-goal-coordinates').html()

    $('#goal-modal-body-content').append(newGoalTemplate)
    $('#goal-modal-body-content .goal')
      .addClass('goal-' + goalCount)
      .removeClass('goal')
    $('.goal-' + goalCount)
      .find('*')
      .addClass('goal-' + goalCount)

    createGoalButtons(goalCount)
    goalCount++
  }

  function goalHandlerTemplate (mode, format, current) {
    $('#goal-' + mode + '-fieldset.goal-' + current).attr('format', format)
    $('#goal-' + mode + '-select-format.goal-' + current).dropdown('toggle')
    $('#goal-' + mode + '-select-format.goal-' + current).detach()

    const inputTemplate = $('#goal-' + format + '-input-template').html()
    $('#goal-' + mode + '-input-group.goal-' + current).append(inputTemplate)
    $('#goal-' + mode + '-input-group.goal-' + current)
      .find('input')
      .addClass('goal-' + current + ' ' + mode)
    $(
      '#goal-' +
        mode +
        '-input-group.goal-' +
        current +
        ' span.input-group-text:first'
    ).text(mode)
  }

  function createGoalButtons (current) {
    const goalButtons = $('#goal-buttons').html()
    const currentGoalButton = '#goal-buttons-input-group.goal-' + current
    $(currentGoalButton).append(goalButtons)
    $(currentGoalButton)
      .find('*')
      .addClass('goal-' + current)

     createGoalsFormatButtonsHandler("button[id^='goal-latitude'].goal-" + current + ", button[id^='goal-longitude'].goal-" + current)
     createGoalsConfirmButtonHandler(current)
     createGoalsDeleteButtonHandler(current)
  }

  function goalConfirmButtonHandler (current) {
    let latitude_format = null
    let longitude_format = null

    latitude_format = $('#goal-latitude-fieldset.goal-' + current).attr(
      'format'
    )
    longitude_format = $('#goal-longitude-fieldset.goal-' + current).attr(
      'format'
    )

    if (latitude_format != null && longitude_format != null) {
      $('#goal-change-btn.goal-' + current).prop('disabled', false)
      $('#goal-confirm-btn.goal-' + current).prop('disabled', true)

      $('#goal-latitude-select-format.goal-' + current).detach()
      $('#goal-longitude-select-format.goal-' + current).detach()

      setGoalData(current, latitude_format, longitude_format)
    } else {
      logWarn('Please select a latitude and longitude format.')
    }
  }

  function goalDeleteButtonHandler (current) {
    /* ------------------------\
    implement ROS goal deleting
    \------------------------ */
    const goalName = $('#goal-name.goal-' + current).val()

    const msg = new ROSLIB.Message({
      data: goalName
    })

    delete_goal_publisher.publish(msg)

    $('.goal-' + current).remove()
  }

  function setGoalData (current, latitude_format, longitude_format) {
    let latitude = null
    let longitude = null

    const goalElem = $('#goal-name.goal-' + current)
    const goalName = goalElem.val()

    try {
      for (let i = 0; i < goalList.length; i++) {
        if (goalName == goalList[i].name) {
          throw 'Enter a different goal name'
        }
      }

      switch (latitude_format) {
        case 'decimal-degrees': {
          latitude = parseFloat(
            $(
              '#goal-decimal-degrees-decimal-degree-input.latitude.goal-' +
                current
            ).val()
          )

          if (isNaN(latitude)) {
            throw 'Bad input latitude for decimal degrees format'
          }

          $(
            '#goal-decimal-degrees-decimal-degree-input.latitude.goal-' +
              current
          ).attr('value', latitude)
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $(
              '#goal-degrees-decimal-minutes-degree-input.latitude.goal-' +
                current
            ).val()
          )
          const minute = parseFloat(
            $(
              '#goal-degrees-decimal-minutes-decimal-minute-input.latitude.goal-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input latitude for degrees-decimal-minutes format'
          }

          $(
            '#goal-degrees-decimal-minutes-degree-input.latitude.goal-' +
              current
          ).attr('value', decimal)
          $(
            '#goal-degrees-decimal-minutes-decimal-minute-input.latitude.goal-' +
              current
          ).attr('value', minute)

          latitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-degree-input.latitude.goal-' +
                current
            ).val()
          )
          const minutes = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-minute-input.latitude.goal-' +
                current
            ).val()
          )
          const seconds = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-second-input.latitude.goal-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input latitude for degrees-minutes-seconds format'
          }

          $(
            '#goal-degrees-minutes-seconds-degee-input.latitude.goal-' + current
          ).attr('value', decimal)
          $(
            '#goal-degrees-minutes-seconds-minute-input.latitude.goal-' +
              current
          ).attr('value', minutes)
          $(
            '#goal-degrees-minutes-seconds-second-input.latitude.goal-' +
              current
          ).attr('value', seconds)

          latitude =
            decimal + (minutes / minuteTodecmial + seconds / secondTodecmial)
          break
        }

        default:
          throw "You didn't enter anything for latitude"
      }

      switch (longitude_format) {
        case 'decimal-degrees': {
          longitude = parseFloat(
            $(
              '#goal-decimal-degrees-decimal-degree-input.longitude.goal-' +
                current
            ).val()
          )

          if (isNaN(longitude)) {
            throw 'Bad input longitude for decimal-degrees format'
          }

          $(
            '#goal-decimal-degrees-decimal-degree-input.longitude.goal-' +
              current
          ).attr('value', longitude)
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $(
              '#goal-degrees-decimal-minutes-degree-input.longitude.goal-' +
                current
            ).val()
          )
          const minute = parseFloat(
            $(
              '#goal-degrees-decimal-minutes-decimal-minute-input.longitude.goal-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input longitude for degrees-decimal-minutes format'
          }

          $(
            '#goal-degrees-decimal-minutes-decimal-input.longitude.goal-' +
              current
          ).attr('value', decimal)
          $(
            '#goal-degrees-decimal-minutes-decimal-minute-input.longitude.goal-' +
              current
          ).attr('value', minute)

          longitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-degree-input.longitude.goal-' +
                current
            ).val()
          )
          const minutes = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-minute-input.longitude.goal-' +
                current
            ).val()
          )
          const seconds = parseFloat(
            $(
              '#goal-degrees-minutes-seconds-second-input.longitude.goal-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input long for degrees-minutes-seconds format'
          }

          $(
            '#goal-degrees-minutes-seconds-degree-input.longitude.goal-' +
              current
          ).attr('value', decimal)
          $(
            '#goal-degrees-minutes-seconds-minute-input.longitude.goal-' +
              current
          ).attr('value', minutes)
          $(
            '#goal-degrees-minutes-seconds-second-input.longitude.goal-' +
              current
          ).attr('value', seconds)

          longitude =
            decimal + (minutes / minuteTodecmial + seconds / secondTodecmial)
          break
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

      /* ----------------------------------\
        this section implements ROS goal node
        \---------------------------------- */
      const goalData = new ROSLIB.Message({
        name: goalName,
        longitude: longitude,
        latitude: latitude
      })

      create_goal_publisher.publish(goalData)

      logInfo('Goal parameters have been set!')
    } catch (e) {
      $('#goal-confirm-btn.goal-' + current).prop('disabled', false)
      $('#goal-change-btn.goal-' + current).prop('disabled', true)

      logWarn(e)
    }
  }

  function updateAntennaParams () {
    antenna_latitude.get(function (latitude) {
      antenna_longitude.get(function (longitude) {
        antenna_start_dir.get(function (heading) {
          if (latitude && longitude && heading) {
            setupAntennaStats(latitude, longitude, heading)
          }
        })
      })
    })
  }
})
