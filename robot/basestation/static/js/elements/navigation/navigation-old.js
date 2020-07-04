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

    goalList = []
    goalCount = 0;

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

    $('#antenna-select-latitude-format-btn').on('click', function(event) {
        event.preventDefault()
        createAntennaLatitudeHandler()
    })
    $('#antenna-select-longitude-format-btn').on('click', function(event) {
        event.preventDefault()
        createAntennaLongitudeHandler()
    })
    $('#antenna-confirm-btn').on('click', function(event) {
        event.preventDefault()

        let latitude = -1
        let longitude = -1

        try {
            if ($("#antenna-latitude-fieldset").attr('format') == 'DD') {
                let decimaldegree = parseFloat(($("#antenna-DD-decimal-degree-input.latitude").val()))
                $("#antenna-DD-decimal-degree-input.latitude").attr("value", decimaldegree)

                if (isNaN(decimaldegree)) {
                  throw "Bad Input latitude DD"
                }

                latitude = decimaldegree
            } else if ($("#antenna-latitude-fieldset").attr('format') == 'DDM') {
                let decimal = parseFloat($('#antenna-DDM-degree-input.latitude').val())
                let minute = parseFloat($('#antenna-DDM-decimal-minute-input.latitude').val())

                if (isNaN(decimal) || isNaN(minute)) {
                  throw "Bad input latitude DDM"
                }

                $("#antenna-DDM-degree-input.latitude").attr("value", decimal)
                $("#antenna-DDM-decimal-minute-input.latitude").attr("value", minute)

                latitude = decimal + (minute / 60)
            } else if ($("#antenna-latitude-fieldset").attr('format') == 'DMS') {
                let decimal = parseFloat($('#antenna-DMS-deg-input.latitude').val())
                let minutes = parseFloat($('#antenna-DMS-minute-input.latitude').val())
                let seconds = parseFloat($('#antenna-DMS-second-input.latitude').val())

                if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) throw "Bad input latitude DMS"

                $("#antenna-DMS-degree-input.latitude").attr("value", decimal)
                $("#antenna-DMS-minute-input.latitude").attr("value", minutes)
                $("#antenna-DMS-second-input.latitude").attr("value", seconds)

                latitude = decimal + (minutes / 60 + seconds / 3600)
            }

            if ($("#antenna-longitude-fieldset").attr('format') == 'DD') {
                let decimaldegree = parseFloat(($("#antenna-DD-decimal-degree-input.longitude").val()))

                if (isNaN(decimaldegree)) {
                  throw "Bad Input longitude DD"
                }

                $("#antenna-DD-decimal-degree-input.longitude").attr("value", decimaldegree)

                longitude = decimaldegree
            } else if ($("#antenna-longitude-fieldset").attr('format') == 'DDM') {
                let decimal = parseFloat($('#antenna-DDM-degree-input.longitude').val())
                let minute = parseFloat($('#antenna-DDM-decimal-minute-input.longitude').val())

                if (isNaN(decimal) || isNaN(minute)) {
                  throw "Bad input longitude DDM"
                }

                $("#antenna-DDM-degree-input.longitude").attr("value", decimal)
                $("#antenna-DDM-decimal-minute-input.longitude").attr("value", minute)

                longitude = decimal + (minute / 60)
            } else if ($("#antenna-longitude-fieldset").attr('format') == 'DMS') {
                let decimal = parseFloat($('#antenna-DMS-degree-input.longitude').val())
                let minutes = parseFloat($('#antenna-DMS-minute-input.longitude').val())
                let seconds = parseFloat($('#antenna-DMS-second-input.longitude').val())

                if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
                  throw "Bad input longitude DMS"
                }

                $("#antenna-DMS-degree-input.longitude").attr("value", decimal)
                $("#antenna-DMS-minute-input.longitude").attr("value", minutes)
                $("#antenna-DMS-second-input.longitude").attr("value", seconds)

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

            createAntennaBearingChangeButtonHandler()

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
    })

    function antennaChangeButtonHandlerTemplate(mode, format, detachedData) {
        $("#antenna-" + format + "-change-btn." + mode).on('click', function(event) {
            event.preventDefault()

            $("#antenna-" + mode + "-input-group").empty()
            $("#antenna-" + mode + "-fieldset").append(detachedData)
            $("#antenna-confirm-btn").prop('disabled', false)
        })
    }

    function createAntennaLatitudeChangeButtonHandler(detachedData) {
        antennaChangeButtonHandlerTemplate("latitude", "DD", detachedData)
        antennaChangeButtonHandlerTemplate("latitude", "DDM", detachedData)
        antennaChangeButtonHandlerTemplate("latitude", "DMS", detachedData)
    }

    function createAntennaLongitudeChangeButtonHandler(detachedData) {
        antennaChangeButtonHandlerTemplate("longitude", "DD", detachedData)
        antennaChangeButtonHandlerTemplate("longitude", "DDM", detachedData)
        antennaChangeButtonHandlerTemplate("longitude", "DMS", detachedData)
    }

    function createAntennaBearingChangeButtonHandler() {
        $("#antenna-bearing-change-btn").on('click', function(event) {
            $("#antenna-bearing-input").prop('disabled', false)
            $("#antenna-bearing-change-btn").prop('disabled', false)
            $("#antenna-confirm-btn").prop('disabled', false)
        })
    }

    function antennaHandlerTemplate(mode, format) {
        $('#antenna-' + mode + '-' + format + '-btn').unbind('click').on('click', function(event) {
            $('#antenna-' + mode + '-fieldset').attr('format', format)
            $('#antenna-select-' + mode + '-format-btn').dropdown('toggle')
            let detachedData = $('#antenna-select-' + mode + '-format').detach()
            $("#antenna-" + mode + "-fieldset").empty()

            let antennaInputTemplate = $("div." + format + "template").html()
            $("#antenna-" + mode + "-fieldset").append(antennaInputTemplate).find('*').addClass(format + " " + mode)
            $("#antenna-" + mode + "-fieldset span." + format + ":first").text(mode)
            $('.antenna-input-field.' + mode).prop('disabled', false)

            if (mode == 'latitude') createAntennaLatitudeChangeButtonHandler(detachedData)
            else createAntennaLongitudeChangeButtonHandler(detachedData)
        })
    }

    function createAntennaLatitudeHandler() {
        antennaHandlerTemplate('latitude', 'DD')
        antennaHandlerTemplate('latitude', 'DDM')
        antennaHandlerTemplate('latitude', 'DMS')
    }

    function createAntennaLongitudeHandler() {
        antennaHandlerTemplate('longitude', 'DD')
        antennaHandlerTemplate('longitude', 'DDM')
        antennaHandlerTemplate('longitude', 'DMS')
    }

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

    $("#new-goal-coordinates-btn").on('click', function(event) {
        let newGoalTemplate = $('#add-goal-coordinates').html()

        $("#goal-modal-body-content").append(newGoalTemplate)
        $("#goal-modal-body-content .goal").addClass('goal-' + goalCount).removeClass('goal')
        $(".goal-" + goalCount).find('*').addClass('goal-' + goalCount)

        createGoalLatitudeHandler(goalCount)
        createGoalLongitudeHandler(goalCount)
        createGoalButtons(goalCount)
        goalCount++
    })

    function saveGoalData(current, latitude_format, longitude_format) {
        let latitude = null
        let longitude = null

        try {
            for (let i = 0; i < goalCount; i++) {
                if ($('#goal-name.goal-' + current).val() == goalList[i].name) {
                    throw 'Enter different name'
                }
            }
            if (latitude_format == 'decimal-degrees') {
                //decimal-degrees mode
                decimaldegree = parseFloat($('#goal-DD-decimal-degree-input.latitude.goal-' + current).val())

                if (isNaN(decimaldegree)) {
                  throw "Bad input latitude for decimal degrees format"
                }

                $('#goal-DD-decimal-degree-input.latitude.goal-' + current).attr("value", decimaldegree)

                latitude = decimaldegree
            } else if (latitude_format == 'degrees-decimal-minutes') {
                //degrees-decimal-minutes mode
                let decimal = parseFloat($('#goal-DDM-degree-input.latitude.goal-' + current).val())
                let minute = parseFloat($('#goal-DDM-decimal-minute-input.latitude.goal-' + current).val())

                if (isNaN(decimal) || isNaN(minute)) {
                  throw "Bad input latitude DDM"
                }

                latitude = decimal + (minute / 60)

                $('#goal-DDM-degree-input.latitude.goal-' + current).attr("value", decimal)
                $('#goal-DDM-decimal-minute-input.latitude.goal-' + current).attr("value", minute)

            } else if (latitude_format == 'degrees-minutes-seconds') {
                let decimal = parseFloat($('#goal-DMS-degree-input.latitude.goal-' + current).val())
                let minutes = parseFloat($('#goal-DMS-minute-input.latitude.goal-' + current).val())
                let seconds = parseFloat($('#goal-DMS-second-input.latitude.goal-' + current).val())

                if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) {
                  throw "Bad input latitude DMS"
                }

                latitude = decimal + (minutes / 60 + seconds / 3600)

                $('#goal-DMS-degee-input.latitude.goal-' + current).attr("value", decimal)
                $('#goal-DMS-minute-input.latitude.goal-' + current).attr("value", minutes)
                $('#goal-DMS-second-input.latitude.goal-' + current).attr("value", seconds)
            } else {
                throw "You didn't enter anything for latitude"
            }
            if (longitude_format == 'decimal-degrees') {
                longitude = parseFloat($('#goal-DD-decimal-degree-input.longitude.goal-' + current).val())

                if (isNaN(longitude)) {
                  throw "Bad input longitude DD"
                }

                $('#goal-DD-decimal-degree-input.longitude.goal-' + current).attr("value", longitude)
            } else if (longitude_format == 'degrees-decimal-minutes') {
                let decimal = parseFloat($('#goal-DDM-degree-input.longitude.goal-' + current).val())
                let minute = parseFloat($('#goal-DDM-decimal-minute-input.longitude.goal-' + current).val())

                if (isNaN(decimal) || isNaN(minute)) {
                  throw "Bad input longitude DDM"
                }

                longitude = decimal + (minute / 60)

                $('#goal-DDM-decimal-input.longitude.goal-' + current).attr("value", decimal)
                $('#goal-DDM-decimal-minute-input.longitude.goal-' + current).attr("value", minute)

            } else if (longitude_format == 'degrees-minutes-seconds') {
                let decimal = parseFloat($('#goal-DMS-degree-input.longitude.goal-' + current).val())
                let minutes = parseFloat($('#goal-DMS-minute-input.longitude.goal-' + current).val())
                let seconds = parseFloat($('#goal-DMS-second-input.longitude.goal-' + current).val())

                if (isNaN(decimal) || isNaN(minutes) || isNaN(seconds)) throw "Bad input long DMS"

                longitude = decimal + (minutes / 60 + seconds / 3600)

                $('#goal-DMS-degree-input.longitude.goal-' + current).attr("value", decimal)
                $('#goal-DMS-minute-input.longitude.goal-' + current).attr("value", minutes)
                $('#goal-DMS-second-input.longitude.goal-' + current).attr("value", seconds)
            } else {
                throw "You didn't enter anything for longitude"
            }

            let goalElem = $('#goal-name.goal-' + current)
            let goalName = goalElem.val()
            if (goalName.length != 0) {
              goalElem.attr('value', goalName)
            }
            else {
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

            create_goal_publisher.publish(goalData)

        } catch (e) {
            $('#goal-confirm-btn.goal-' + current).prop('disabled', false)
            $('#goal-change-btn.goal-' + current).prop('disabled', true)

            logWarn(e)
        }
    }

    function goalHandlerTemplate(mode, format, current) {
        $("#goal-" + mode + "-" + format + "-btn.goal-" + current).on('click', function(event) {
            event.preventDefault()

            $('#goal-' + mode + '-fieldset.goal-' + current).attr("format", format)
            $('#goal-' + mode + '-select-format.goal-' + current).dropdown('toggle')
            $('#goal-' + mode + '-select-format.goal-' + current).detach()

            let inputTemplate = $('#goal-' + format + '-input-template').html()
            $('#goal-' + mode + '-input-group.goal-' + current).append(inputTemplate)
            $('#goal-' + mode + '-input-group.goal-' + current).find('input').addClass('goal-' + current + ' ' + mode)
            $('#goal-' + mode + '-input-group.goal-' + current + ' span.input-group-text:first').text(mode)
        })
    }

    function createGoalLatitudeHandler(current) {
        goalHandlerTemplate('latitude', 'DD', current)
        goalHandlerTemplate('latitude', 'DDM', current)
        goalHandlerTemplate('latitude', 'DMS', current)
    }

    function createGoalLongitudeHandler(current) {
        goalHandlerTemplate('longitude', 'DD', current)
        goalHandlerTemplate('longitude', 'DDM', current)
        goalHandlerTemplate('longitude', 'DMS', current)
    }

    function createGoalButtons(current) {
        let goalButtons = $("#goal-buttons").html()
        $("#goal-buttons-input-group.goal-" + current).append(goalButtons)
        $("#goal-buttons-input-group.goal-" + current).find('*').addClass('goal-' + current)

        createGoalConfirmButtonHandler(current)
        //createGoalChangeButtonHandler(current)
        createGoalDeleteButtonHandler(current)
    }

    function createGoalConfirmButtonHandler(current) {
        $('#goal-confirm-btn.goal-' + current).on('click', function(event) {
            let latitude_format = null
            let longitude_format = null

            if ($("#goal-latitude-fieldset.goal-" + current).attr("format") == 'DD')
                latitude_format = 'decimal-degrees'
            else if ($("#goal-latitude-fieldset.goal-" + current).attr("format") == 'DDM')
                latitude_format = 'degrees-decimal-minutes'
            else if ($("#goal-latitude-fieldset.goal-" + current).attr("format") == 'DMS')
                latitude_format = 'degree-minutes-seconds'
            else logErr('Error with goal latitude format')

            if ($("#goal-longitude-fieldset.goal-" + current).attr("format") == 'DD')
                longitude_format = 'decimal-degrees'
            else if ($("#goal-longitude-fieldset.goal-" + current).attr("format") == 'DDM')
                longitude_format = 'degrees-decimal-minutes'
            else if ($("#goal-longitude-fieldset.goal-" + current).attr("format") == 'DMS')
                longitude_format = 'degree-minutes-seconds'
            else logErr('Error with goal longitude format')

            $('#goal-change-btn.goal-' + current).prop('disabled', false)
            $('#goal-confirm-btn.goal-' + current).prop('disabled', true)

            $('#goal-latitude-select-format.goal-' + current).detach()
            $('#goal-longitude-select-format.goal-' + current).detach()

            saveGoalData(current, latitude_format, longitude_format)
        })
    }

    function createGoalChangeButtonHandler(current) {
        $('#goal-change-btn.goal-' + current).on('click', function(event) {
            $(this).data('clicked', true)

            $('.goal-' + current + ' fieldset').prop('disabled', false)

            $('#goal-change-btn.goal-' + current).prop('disabled', true)
            $('#goal-confirm-btn.goal-' + current).prop('disabled', false)
        })
    }

    function createGoalDeleteButtonHandler(current) {
        $('#goal-delete-btn.goal-' + current).on('click', function(event) {

            /*------------------------\
            implement ROS goal deleting
            \------------------------*/
            let goalName = $('#goal-name.goal-' + current).val()
            let msg = new ROSLIB.Message({
                data: goalName
            })
            delete_goal_publisher.publish(msg)

            $('.goal-' + current).remove()
        })
    }

    function updateAntennaParams() {
        antenna_latitude.get(function(lat) {
            antenna_longitude.get(function(long) {
                antenna_start_dir.get(function(heading) {
                    if (latitude && longitude && heading) {
                        $('#antenna-stats-latitude').text(latitude.toFixed(6))
                        $('#antenna-stats-longitude').text(longitude.toFixed(6))
                        $('#antenna-stats-heading').text(heading + '°')
                    }
                })
            })
        })
    }
})
