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
            goal_latitude.set(goalList[0].lat)
            goal_longitude.set(goalList[0].long)
            $('#goal-stats-lat').text(goalList[0].lat.toFixed(6))
            $('#goal-stats-long').text(goalList[0].long.toFixed(6))
        } else {
            goal_latitude.set(false)
            goal_longitude.set(false)
            $('#goal-stats-lat').text('----')
            $('#goal-stats-long').text('----')
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
        antenna_latitude.get(function(lat) {
            antenna_longitude.get(function(long) {
                antenna_start_dir.get(function(heading) {
                    if (lat && long && heading) {
                        logInfo('Found antenna parameters to set')
                        setupAntennaStats(lat, long, heading)
                    } else {
                        logInfo('Antenna parameters were not found')
                    }
                })
            })
        })

        goal_latitude.get(function(lat) {
            goal_longitude.get(function(long) {
                if (lat && long) {
                    has_gps_goal.set(true)
                    logInfo('Found goal coordinates to set')
                    setupGoalStats(lat, long)
                } else {
                    logInfo('Goal parameters not found')
                }
            })
        })

        function setupAntennaStats(lat, long, heading) {
            $('#antenna-stats-lat').text(lat.toFixed(6))
            $('#antenna-stats-long').text(long.toFixed(6))
            $('#antenna-stats-heading').text(heading + '°')
        }

        function setupGoalStats(lat, long) {
            $('#goal-stats-lat').text(lat.toFixed(6))
            $('#goal-stats-long').text(long.toFixed(6))
        }
    }

    $('#antenna-select-lat-format-btn').on('click', function(event) {
        event.preventDefault()
        createAntennaLatitudeHandler()
    })
    $('#antenna-select-long-format-btn').on('click', function(event) {
        event.preventDefault()
        createAntennaLongitudeHandler()
    })
    $('#antenna-confirm-btn').on('click', function(event) {
        event.preventDefault()

        let lat = -1
        let long = -1

        try {
            if ($("#antenna-lat-fieldset").attr('format') == 'DD') {
                let decdeg = parseFloat(($("#antenna-DD-dec-deg-input.lat").val()))
                $("#antenna-DD-dec-deg-input.lat").attr("value", decdeg)

                if (isNaN(decdeg)) throw "Bad Input lat DD"

                lat = decdeg
            } else if ($("#antenna-lat-fieldset").attr('format') == 'DDM') {
                let dec = parseFloat($('#antenna-DDM-deg-input.lat').val())
                let min = parseFloat($('#antenna-DDM-dec-min-input.lat').val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"

                $("#antenna-DDM-deg-input.lat").attr("value", dec)
                $("#antenna-DDM-dec-min-input.lat").attr("value", min)

                lat = dec + (min / 60)
            } else if ($("#antenna-lat-fieldset").attr('format') == 'DMS') {
                let dec = parseFloat($('#antenna-DMS-deg-input.lat').val())
                let mins = parseFloat($('#antenna-DMS-min-input.lat').val())
                let secs = parseFloat($('#antenna-DMS-sec-input.lat').val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS"

                $("#antenna-DMS-deg-input.lat").attr("value", dec)
                $("#antenna-DMS-min-input.lat").attr("value", mins)
                $("#antenna-DMS-sec-input.lat").attr("value", secs)

                lat = dec + (mins / 60 + secs / 3600)
            }

            if ($("#antenna-long-fieldset").attr('format') == 'DD') {
                let decdeg = parseFloat(($("#antenna-DD-dec-deg-input.long").val()))

                if (isNaN(decdeg)) throw "Bad Input long DD"

                $("#antenna-DD-dec-deg-input.long").attr("value", decdeg)

                long = decdeg
            } else if ($("#antenna-long-fieldset").attr('format') == 'DDM') {
                let dec = parseFloat($('#antenna-DDM-deg-input.long').val())
                let min = parseFloat($('#antenna-DDM-dec-min-input.long').val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input long DDM"

                $("#antenna-DDM-deg-input.long").attr("value", dec)
                $("#antenna-DDM-dec-min-input.long").attr("value", min)

                long = dec + (min / 60)
            } else if ($("#antenna-long-fieldset").attr('format') == 'DMS') {
                let dec = parseFloat($('#antenna-DMS-deg-input.long').val())
                let mins = parseFloat($('#antenna-DMS-min-input.long').val())
                let secs = parseFloat($('#antenna-DMS-sec-input.long').val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS"

                $("#antenna-DMS-deg-input.long").attr("value", dec)
                $("#antenna-DMS-min-input.long").attr("value", mins)
                $("#antenna-DMS-sec-input.long").attr("value", secs)

                long = dec + (mins / 60 + secs / 3600)
            }
            let bearing = parseFloat($("#antenna-bearing-input").val())
            if (isNaN(bearing)) throw "Bad input bearing"

            $("#antenna-bearing-input").attr("value", bearing)


            $('.antenna-input-field').prop('disabled', true)
            $('.antenna-change-btn').prop('disabled', false)
            $('#antenna-confirm-btn').prop('disabled', true)

            createAntennaBearingChangeButtonHandler()

            //ROS params
            antenna_latitude.set(lat)
            antenna_longitude.set(long)
            antenna_start_dir.set(bearing)
            $('#antenna-stats-lat').text(lat.toFixed(6))
            $('#antenna-stats-long').text(long.toFixed(6))
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
        antennaChangeButtonHandlerTemplate("lat", "DD", detachedData)
        antennaChangeButtonHandlerTemplate("lat", "DDM", detachedData)
        antennaChangeButtonHandlerTemplate("lat", "DMS", detachedData)
    }

    function createAntennaLongitudeChangeButtonHandler(detachedData) {
        antennaChangeButtonHandlerTemplate("long", "DD", detachedData)
        antennaChangeButtonHandlerTemplate("long", "DDM", detachedData)
        antennaChangeButtonHandlerTemplate("long", "DMS", detachedData)
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

            if (mode == 'lat') createAntennaLatitudeChangeButtonHandler(detachedData)
            else createAntennaLongitudeChangeButtonHandler(detachedData)
        })
    }

    function createAntennaLatitudeHandler() {
        antennaHandlerTemplate('lat', 'DD')
        antennaHandlerTemplate('lat', 'DDM')
        antennaHandlerTemplate('lat', 'DMS')
    }

    function createAntennaLongitudeHandler() {
        antennaHandlerTemplate('long', 'DD')
        antennaHandlerTemplate('long', 'DDM')
        antennaHandlerTemplate('long', 'DMS')
    }

    $('#toggle-goal-modal-btn').on('click', function(event) {
        $("#goal-modal-body-content").empty()
        let goalTemplate = $('#created-goal-template').html()
        for (i = 0; i < goalCount; i++) {
            $("#goal-modal-body-content").append(goalTemplate)
            $("#goal-modal-body-content .goal").addClass('goal-' + i).removeClass('goal')
            $(".goal-" + i).find('*').addClass('goal-' + i)

            createGoalButtons(i)

            $('#goal-name.goal-' + i).val(goalList[i].name)
            $('#goal-confirm-btn.goal-' + i).prop('disabled', true)
            $('#goal-change-btn.goal-' + i).prop('disabled', false)
            $('div.goal-' + i + ' fieldset').prop('disabled', true)
            $('#goal-DD-dec-deg-input.lat.goal-' + i).val(goalList[i].lat)
            $('#goal-DD-dec-deg-input.long.goal-' + i).val(goalList[i].long)

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
    })

    function saveGoalData(current, lat_format, long_format) {
        let lat = null
        let long = null
        try {
            for (i = 0; i < goalCount; i++) {
                if ($('#goal-name.goal-' + current).val() == goalList[i].name) {
                    throw 'Enter different name'
                }
            }
            if (lat_format == 0) {
                //decimal-degrees mode
                decdeg = parseFloat($('#goal-DD-dec-deg-input.lat.goal-' + current).val())

                if (isNaN(decdeg)) throw "Bad input lat DD"

                $('#goal-DD-dec-deg-input.lat.goal-' + current).attr("value", decdeg)

                lat = decdeg
            } else if (lat_format == 1) {
                //degrees-decimal-minutes mode
                let dec = parseFloat($('#goal-DDM-deg-input.lat.goal-' + current).val())
                let min = parseFloat($('#goal-DDM-dec-min-input.lat.goal-' + current).val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"

                lat = dec + (min / 60)

                $('#goal-DDM-deg-input.lat.goal-' + current).attr("value", dec)
                $('#goal-DDM-dec-min-input.lat.goal-' + current).attr("value", min)

            } else if (lat_format == 2) {
                //degrees-minutes-seconds mode
                let dec = parseFloat($('#goal-DMS-deg-input.lat.goal-' + current).val())
                let mins = parseFloat($('#goal-DMS-min-input.lat.goal-' + current).val())
                let secs = parseFloat($('#goal-DMS-sec-input.lat.goal-' + current).val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS"

                lat = dec + (mins / 60 + secs / 3600)

                $('#goal-DMS-deg-input.lat.goal-' + current).attr("value", dec)
                $('#goal-DMS-min-input.lat.goal-' + current).attr("value", mins)
                $('#goal-DMS-sec-input.lat.goal-' + current).attr("value", secs)
            } else {
                throw "You didn't enter anything for lat"
            }
            if (long_format == 0) {
                //decimal-degrees mode
                long = parseFloat($('#goal-DD-dec-deg-input.long.goal-' + current).val())

                if (isNaN(long)) throw "Bad input long DD"

                $('#goal-DD-dec-deg-input.long.goal-' + current).attr("value", long)
            } else if (long_format == 1) {
                //degrees-decimal-minutes mode
                let dec = parseFloat($('#goal-DDM-deg-input.long.goal-' + current).val())
                let min = parseFloat($('#goal-DDM-dec-min-input.long.goal-' + current).val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input long DDM"

                long = dec + (min / 60)

                $('#goal-DDM-deg-input.long.goal-' + current).attr("value", dec)
                $('#goal-DDM-dec-min-input.long.goal-' + current).attr("value", min)

            } else if (long_format == 2) {
                //degrees-minutes-seconds mode
                let dec = parseFloat($('#goal-DMS-deg-input.long.goal-' + current).val())
                let mins = parseFloat($('#goal-DMS-min-input.long.goal-' + current).val())
                let secs = parseFloat($('#goal-DMS-sec-input.long.goal-' + current).val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS"

                long = dec + (mins / 60 + secs / 3600)

                $('#goal-DMS-deg-input.long.goal-' + current).attr("value", dec)
                $('#goal-DMS-min-input.long.goal-' + current).attr("value", mins)
                $('#goal-DMS-sec-input.long.goal-' + current).attr("value", secs)
            } else {
                throw "You didn't enter anything for long"
            }

            let goalElem = $('#goal-name.goal-' + current)
            let goalName = goalElem.val()
            if (goalName.length != 0) goalElem.attr('value', goalName)
            else goalElem.attr('value', 'Goal-' + current)

            $('#goal-name-fieldset.goal-' + current).prop('disabled', true)
            $('#goal-lat-fieldset.goal-' + current).prop('disabled', true)
            $('#goal-long-fieldset.goal-' + current).prop('disabled', true)


            /*----------------------------------\
            this section implements ROS goal node
            \----------------------------------*/
            let goalData = new ROSLIB.Message({
                name: goalName,
                long: long,
                lat: lat
            })

            create_goal_publisher.publish(goalData)

        } catch (e) {
            $('#goal-confirm-btn.goal-' + current).prop('disabled', false)
            $('#goal-change-btn.goal-' + current).prop('disabled', true)

            rosLog('ROSWARN', e)
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
        goalHandlerTemplate('lat', 'DD', current)
        goalHandlerTemplate('lat', 'DDM', current)
        goalHandlerTemplate('lat', 'DMS', current)
    }

    function createGoalLongitudeHandler(current) {
        goalHandlerTemplate('long', 'DD', current)
        goalHandlerTemplate('long', 'DDM', current)
        goalHandlerTemplate('long', 'DMS', current)
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
            let lat_format = -1
            let long_format = -1

            if ($("#goal-lat-fieldset.goal-" + current).attr("format") == 'DD')
                lat_format = 0
            else if ($("#goal-lat-fieldset.goal-" + current).attr("format") == 'DDM')
                lat_format = 1
            else if ($("#goal-lat-fieldset.goal-" + current).attr("format") == 'DMS')
                lat_format = 2
            else logErr('Error with goal latitude format')

            if ($("#goal-long-fieldset.goal-" + current).attr("format") == 'DD')
                long_format = 0
            else if ($("#goal-long-fieldset.goal-" + current).attr("format") == 'DDM')
                long_format = 1
            else if ($("#goal-long-fieldset.goal-" + current).attr("format") == 'DMS')
                long_format = 2
            else logErr('Error with goal longitude format')

            $('#goal-change-btn.goal-' + current).prop('disabled', false)
            $('#goal-confirm-btn.goal-' + current).prop('disabled', true)

            $('#goal-lat-select-format.goal-' + current).detach()
            $('#goal-long-select-format.goal-' + current).detach()

            saveGoalData(current, lat_format, long_format)
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
                    if (lat && long && heading) {
                        $('#antenna-stats-lat').text(lat.toFixed(6))
                        $('#antenna-stats-long').text(long.toFixed(6))
                        $('#antenna-stats-heading').text(heading + '°')
                    } 
                })
            })
        })
    }
})


