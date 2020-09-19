$(document).ready(() => {
  markerList = []
  markerCount = 0

  markerListBackupsCount = 0
  const maxmarkerListBackups = 3
  settingmarkerAsCurrent = false

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
  // setup a subscriber for the antenna_marker topic
  const antenna_marker_listener = new ROSLIB.Topic({
    name: 'antenna_marker',
    messageType: 'geometry_msgs/Point',
    ros: ros
  })
  // setup topics to communicate with markersNode
  const create_marker_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'create_marker',
    messageType: 'mcu_control/RoverMarker'
  })

  const delete_marker_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'delete_marker',
    messageType: 'std_msgs/String'
  })

  const set_as_current_marker_publisher = new ROSLIB.Topic({
    ros: ros,
    name: 'set_as_current_marker',
    messageType: 'std_msgs/String'
  })

  marker_list_subscriber.subscribe(function (message) {
    if (JSON.stringify(markerList) != JSON.stringify(message.marker_list)) {
      markerList = message.marker_list

      toggleMarkers()

      if (message.marker_list.length != 0) {
        $('#marker-stats-name').text(markerList[0].name)
        $('#marker-stats-latitude').text(markerList[0].latitude.toFixed(6))
        $('#marker-stats-longitude').text(markerList[0].longitude.toFixed(6))
      } else {
        $('#marker-stats-name').text('----')
        $('#marker-stats-latitude').text('----')
        $('#marker-stats-longitude').text('----')
      }
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
  const has_gps_marker = new ROSLIB.Param({
    ros: ros,
    name: 'has_gps_marker'
  })
  // setup a subscriber for the rover_marker topic
  new ROSLIB.Topic({
    ros: ros,
    name: 'rover_marker',
    messageType: 'geometry_msgs/Point'
  })

  function setupAntennaStats (latitude, longitude, heading) {
    $('#antenna-stats-latitude').text(latitude.toFixed(6))
    $('#antenna-stats-longitude').text(longitude.toFixed(6))
    $('#antenna-stats-heading').text(heading + '°')
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

  // set markers info
  // mouseup events for markers data entry buttons
  createmarkersAddButtonHandler()

  function createmarkersFormatButtonsHandler (button) {
      // select latitude or longitude input format buttons
      $(button).mouseup(e => {
          const buttonId = $(e.target).attr('id').split('-')
          const buttonClass = $(e.target).attr('class').split(' ')
          const mode = buttonId[1]
          let format = buttonId[2] + '-' + buttonId[3]
          const markerNum = buttonClass[3].replace('marker-', '')

          if (buttonId[4] != 'btn') {
            format = format + '-' + buttonId[4]
          }

          markerHandlerTemplate(mode, format, markerNum)
      })
  }

  function createmarkersAddButtonHandler () {
      // add new marker templates
      $("button[id='marker-new-coordinates-btn']").mouseup(e => {
          addmarker()
      })
  }

  function createmarkersConfirmButtonHandler (markerNum) {
      // add new marker templates
      $('#marker-confirm-btn.marker-' + markerNum).mouseup(e => {
          markerConfirmButtonHandler(markerNum)
      })
  }

  function createmarkersDeleteButtonHandler (markerNum) {
      // add new marker templates
      $('#marker-delete-btn.marker-' + markerNum).mouseup(e => {
          markerDeleteButtonHandler(markerNum)
      })
  }

  function createmarkersSetAsCurrentButtonHandler (markerNum) {
      // add new marker templates
      $('#marker-set-as-current-btn.marker-' + markerNum).mouseup(e => {
          markerSetAsCurrentButtonHandler(markerNum)
      })
  }

  // functions
  function toggleMarkers() {
    $('div[class*="marker-"]').remove()
    markerCount = markerList.length
    const markerTemplate = $('#created-marker-template').html()
    for (let i = 0; i < markerCount; i++) {
      $('#marker-modal-body-content').append(markerTemplate)
      $('#marker-modal-body-content .marker')
        .addClass('marker-' + i)
        .removeClass('marker')
      $('.marker-' + i)
        .find('*')
        .addClass('marker-' + i)

      createmarkerButtons(i)

      $('#marker-name.marker-' + i).val(markerList[i].name)
      $('#marker-confirm-btn.marker-' + i).prop('disabled', true)
      $('#marker-change-btn.marker-' + i).prop('disabled', false)
      $('div.marker-' + i + ' fieldset').prop('disabled', true)
      $('#marker-decimal-degrees-decimal-degree-input.latitude.marker-' + i).val(markerList[i].latitude)
      $('#marker-decimal-degrees-decimal-degree-input.longitude.marker-' + i).val(markerList[i].longitude)
    }
  }

  function addmarker () {
    const newmarkerTemplate = $('#add-marker-coordinates').html()

    $('#marker-modal-body-content').append(newmarkerTemplate)
    $('#marker-modal-body-content .marker')
      .addClass('marker-' + markerCount)
      .removeClass('marker')
    $('.marker-' + markerCount)
      .find('*')
      .addClass('marker-' + markerCount)

    createmarkerButtons(markerCount)
    markerCount++
  }

  function markerHandlerTemplate (mode, format, current) {
    $('#marker-' + mode + '-fieldset.marker-' + current).attr('format', format)
    $('#marker-' + mode + '-select-format.marker-' + current).dropdown('toggle')
    $('#marker-' + mode + '-select-format.marker-' + current).detach()

    const inputTemplate = $('#marker-' + format + '-input-template').html()
    $('#marker-' + mode + '-input-group.marker-' + current).append(inputTemplate)
    $('#marker-' + mode + '-input-group.marker-' + current)
      .find('input')
      .addClass('marker-' + current + ' ' + mode)
    $(
      '#marker-' +
        mode +
        '-input-group.marker-' +
        current +
        ' span.input-group-text:first'
    ).text(mode)
  }

  function createmarkerButtons (current) {
    const markerButtons = $('#marker-buttons').html()
    const currentmarkerButton = '#marker-buttons-input-group.marker-' + current
    $(currentmarkerButton).append(markerButtons)
    $(currentmarkerButton)
      .find('*')
      .addClass('marker-' + current)

     createmarkersFormatButtonsHandler("button[id^='marker-latitude'].marker-" + current + ", button[id^='marker-longitude'].marker-" + current)
     createmarkersConfirmButtonHandler(current)
     createmarkersDeleteButtonHandler(current)
     createmarkersSetAsCurrentButtonHandler(current)
  }

  function markerConfirmButtonHandler (current) {
    let latitude_format = null
    let longitude_format = null

    latitude_format = $('#marker-latitude-fieldset.marker-' + current).attr(
      'format'
    )
    longitude_format = $('#marker-longitude-fieldset.marker-' + current).attr(
      'format'
    )

    if (latitude_format != null && longitude_format != null) {
      $('#marker-change-btn.marker-' + current).prop('disabled', false)
      $('#marker-confirm-btn.marker-' + current).prop('disabled', true)

      $('#marker-latitude-select-format.marker-' + current).detach()
      $('#marker-longitude-select-format.marker-' + current).detach()

      setmarkerData(current, latitude_format, longitude_format)
    } else {
      logWarn('Please select a latitude and longitude format.')
    }
  }

  function markerDeleteButtonHandler (current) {
    /* ------------------------\
    implement ROS marker deleting
    \------------------------ */
    const markerName = $('#marker-name.marker-' + current).val()

    const msg = new ROSLIB.Message({
      data: markerName
    })

    delete_marker_publisher.publish(msg)
  }

  function markerSetAsCurrentButtonHandler (current) {
      /* ------------------------\
      implement ROS marker setting as current
      \------------------------ */
      const markerName = $('#marker-name.marker-' + current).val()

      const msg = new ROSLIB.Message({
        data: markerName
      })

      set_as_current_marker_publisher.publish(msg)
  }

  function setmarkerData (current, latitude_format, longitude_format) {
    let latitude = null
    let longitude = null

    const markerElem = $('#marker-name.marker-' + current)
    const markerName = markerElem.val()

    try {
      for (let i = 0; i < markerList.length; i++) {
        if (markerName == markerList[i].name) {
          throw 'Enter a different marker name'
        }
      }

      switch (latitude_format) {
        case 'decimal-degrees': {
          latitude = parseFloat(
            $(
              '#marker-decimal-degrees-decimal-degree-input.latitude.marker-' +
                current
            ).val()
          )

          if (isNaN(latitude)) {
            throw 'Bad input latitude for decimal degrees format'
          }

          $(
            '#marker-decimal-degrees-decimal-degree-input.latitude.marker-' +
              current
          ).attr('value', latitude)
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $(
              '#marker-degrees-decimal-minutes-degree-input.latitude.marker-' +
                current
            ).val()
          )
          const minute = parseFloat(
            $(
              '#marker-degrees-decimal-minutes-decimal-minute-input.latitude.marker-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input latitude for degrees-decimal-minutes format'
          }

          $(
            '#marker-degrees-decimal-minutes-degree-input.latitude.marker-' +
              current
          ).attr('value', decimal)
          $(
            '#marker-degrees-decimal-minutes-decimal-minute-input.latitude.marker-' +
              current
          ).attr('value', minute)

          latitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-degree-input.latitude.marker-' +
                current
            ).val()
          )
          const minutes = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-minute-input.latitude.marker-' +
                current
            ).val()
          )
          const seconds = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-second-input.latitude.marker-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input latitude for degrees-minutes-seconds format'
          }

          $(
            '#marker-degrees-minutes-seconds-degee-input.latitude.marker-' + current
          ).attr('value', decimal)
          $(
            '#marker-degrees-minutes-seconds-minute-input.latitude.marker-' +
              current
          ).attr('value', minutes)
          $(
            '#marker-degrees-minutes-seconds-second-input.latitude.marker-' +
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
              '#marker-decimal-degrees-decimal-degree-input.longitude.marker-' +
                current
            ).val()
          )

          if (isNaN(longitude)) {
            throw 'Bad input longitude for decimal-degrees format'
          }

          $(
            '#marker-decimal-degrees-decimal-degree-input.longitude.marker-' +
              current
          ).attr('value', longitude)
          break
        }

        case 'degrees-decimal-minutes': {
          const decimal = parseFloat(
            $(
              '#marker-degrees-decimal-minutes-degree-input.longitude.marker-' +
                current
            ).val()
          )
          const minute = parseFloat(
            $(
              '#marker-degrees-decimal-minutes-decimal-minute-input.longitude.marker-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minute)) {
            throw 'Bad input longitude for degrees-decimal-minutes format'
          }

          $(
            '#marker-degrees-decimal-minutes-decimal-input.longitude.marker-' +
              current
          ).attr('value', decimal)
          $(
            '#marker-degrees-decimal-minutes-decimal-minute-input.longitude.marker-' +
              current
          ).attr('value', minute)

          longitude = decimal + minute / minuteTodecmial
          break
        }

        case 'degrees-minutes-seconds': {
          const decimal = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-degree-input.longitude.marker-' +
                current
            ).val()
          )
          const minutes = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-minute-input.longitude.marker-' +
                current
            ).val()
          )
          const seconds = parseFloat(
            $(
              '#marker-degrees-minutes-seconds-second-input.longitude.marker-' +
                current
            ).val()
          )

          if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
            throw 'Bad input long for degrees-minutes-seconds format'
          }

          $(
            '#marker-degrees-minutes-seconds-degree-input.longitude.marker-' +
              current
          ).attr('value', decimal)
          $(
            '#marker-degrees-minutes-seconds-minute-input.longitude.marker-' +
              current
          ).attr('value', minutes)
          $(
            '#marker-degrees-minutes-seconds-second-input.longitude.marker-' +
              current
          ).attr('value', seconds)

          longitude =
            decimal + (minutes / minuteTodecmial + seconds / secondTodecmial)
          break
        }

        default:
          throw "You didn't enter anything for longitude"
      }

      if (markerName.length != 0) {
        markerElem.attr('value', markerName)
      } else {
        markerElem.attr('value', 'marker-' + current)
      }

      $('#marker-name-fieldset.marker-' + current).prop('disabled', true)
      $('#marker-latitude-fieldset.marker-' + current).prop('disabled', true)
      $('#marker-longitude-fieldset.marker-' + current).prop('disabled', true)

      /* ----------------------------------\
        this section implements ROS marker node
        \---------------------------------- */
      const markerData = new ROSLIB.Message({
        name: markerName,
        color: getRandomColor(),
        longitude: longitude,
        latitude: latitude
      })

      create_marker_publisher.publish(markerData)

      logInfo('marker parameters have been set!')
    } catch (e) {
      $('#marker-confirm-btn.marker-' + current).prop('disabled', false)
      $('#marker-change-btn.marker-' + current).prop('disabled', true)

      logWarn(e)
    }
  }

  function updateAntennaParams () {
    antenna_latitude.get(function (latitude) {
      antenna_longitude.get(function (longitude) {
        antenna_start_dir.get(function (heading) {
          if (latitude != null && longitude != null && heading != null) {
            setupAntennaStats(latitude, longitude, heading)
          }
        })
      })
    })
  }
})
