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
  detachedData = null

  // add antenna data
  // onClick events for antenna data entry buttons
  $("button[id^='antenna-']").mouseup(e => {
    event.preventDefault()
    let buttonId = $(e.target).attr("id").split("-")
    let buttonClass = $(e.target).attr("class").split(" ")
    console.log(buttonClass[2] + ': test')
    if (buttonId[1] == 'longitude' || buttonId[1] == 'latitude') {
      let mode = buttonId[1]
      let format = buttonId[2] + "-" + buttonId[3]
      if (buttonId[4] != 'btn') {
        format = format + "-" + buttonId[4]
      }
      antennaHandlerTemplate(mode, format)
    } else if (buttonClass[2] == 'antenna-change-btn') {
        if (buttonId[1] != 'bearing') {
          let mode = buttonClass[4]
          let format = buttonId[1] + "-" + buttonId[2]
          if (buttonId[3] != 'change') {
            format = format + "-" + buttonId[3]
          }
          console.log(mode + " : " + format)
          antennaChangeButtonHandlerTemplate(mode, format)
        } else {
          //do something
        }
    }
  })

  function antennaHandlerTemplate(mode, format) {
    $('#antenna-' + mode + '-fieldset').attr('format', format)
    $('#antenna-select-' + mode + '-format-btn').dropdown('toggle')
    detachedData = $('#antenna-select-' + mode + '-format').detach()
    $("#antenna-" + mode + "-fieldset").empty()

    let antennaInputTemplate = $("div." + format + "-template").html()
    $("#antenna-" + mode + "-fieldset").append(antennaInputTemplate).find('*').addClass(format + " " + mode)
    $("#antenna-" + mode + "-fieldset span." + format + ":first").text(mode)
    $('.antenna-input-field.' + mode).prop('disabled', false)
  }

  function antennaChangeButtonHandlerTemplate(mode, format) {
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

  // add goals data
  $('#toggle-goal-modal-btn').on('click', function(event) {
    $("#goal-modal-body-content").empty()
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
        $('#goal-DD-decimal-degree-input.latitude.goal-' + i).val(goalList[i].latitude)
        $('#goal-DD-decimal-degree-input.longitude.goal-' + i).val(goalList[i].longitude)

    }
  })

})
