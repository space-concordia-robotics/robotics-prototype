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
        saveAntennaStatsToServer()
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
    // let got_antenna_pos = new ROSLIB.Param({
    //     ros: ros,
    //     name: 'got_antenna_pos'
    // })
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
    let rover_goal_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'rover_goal',
        messageType: 'geometry_msgs/Point'
    })

    rover_goal_listener.subscribe(function(message) {
        //the distance in meters until we are sure we are at the coordinates should be less than 0.5 meters
        if (parseFloat(message.y) < 0.5) {

            // delete the closest goal
            // update goal ROS parameters

            // if nothing is left, display -----

            // $('#goal-stats-lat').text('----')
            // $('#goal-stats-long').text('----')

            // and set the has_gps_goal param to false
            // has_gps_goal.set(false)
        }
    })

    let count = 0
    let goalList = []

    initNavigationPanel()

    function initNavigationPanel() {
        antenna_latitude.get(function(lat) {
            antenna_longitude.get(function(long) {
                antenna_start_dir.get(function(heading) {
                    if (lat && long && heading) {
                        appendToConsole('Antenna parameters already set')
                        //got_antenna_pos.set(true)
                        setupAntennaStats(lat, long, heading)
                    } else {
                        appendToConsole('Enter all antenna parameters')
                        //got_antenna_pos.set(false)
                    }
                })
            })
        })

        goal_latitude.get(function(lat) {
            goal_longitude.get(function(long) {
                if (lat && long) {
                    has_gps_goal.set(true)
                    appendToConsole('Goal coordinates already set')
                    setupGoalStats(lat, long)
                } else {
                    appendToConsole('Add a goal to get nav data')
                }
            })
        })

        // $.get({
        //     url: '/navigation/cached_content/antenna_modal',
        //     success: function(result) {
        //
        //         let detachedLatFormatData = $('#antenna-select-lat-format').detach()
        //         let detachedLongFormatData = $('#antenna-select-long-format').detach()
        //
        //         $('#antenna-modal-body').empty()
        //         $('#antenna-modal-body').append(result)
        //
        //         createAntennaLatitudeChangeButtonHandler(detachedLatFormatData)
        //         createAntennaLongitudeChangeButtonHandler(detachedLongFormatData)
        //         createAntennaBearingChangeButtonHandler()
        //     }
        // })
        function setupAntennaStats(lat, long, heading) {
                $('#antenna-stats-lat').text(lat.toFixed(6))
                $('#antenna-stats-long').text(long.toFixed(6))
                $('#antenna-stats-heading').text(heading + '°')
        }

        function setupGoalStats(lat, long) {
                $('#goal-stats-lat').text(lat)
                $('#goal-stats-long').text(long)
        }


        // we want to build the goal-modal-body body using the current goal list.        $('#goal-modal-body').append()
        // then we want to create the handlers with the appropriate ids.
        // the ids will always start at 0 since this is just for the ui and the ROS thing works by name.

        // these are the button handlers
        // instead of going to count we will go up to the length of the goal list

        // for (i = 0; i < count; i++) {
        //     if ($('#new-goal-btn-' + i).length) {
        //         createGoalLatitudeHandler(i)
        //         createGoalLongitudeHandler(i)
        //         createGoalChangeButtonHandler(i)
        //         createGoalDeleteButtonHandler(i)
        //         createGoalConfirmButtonHandler(i)
        //     }
        // }


        // -------------------------------------------------------------
        // for the goal stats we will generate them dynamically from a default of '----'
        // no need of saving anything...
        // this will use the navigationNode
        // basically it checks if has_gps_goal is true, and returns some stats about the bearing and distance

        // $("#goal-stats-rover-heading").text('----')
        // $("#goal-stats-distance").text('----')
        //
        // $('#goal-stats-lat').text()
        // $('#goal-stats-long').text()
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
                let decdeg = parseFloat(($("#antenna-lat-DD-dec-deg-input").val()))
                $("#antenna-lat-DD-dec-deg-input").attr("value", decdeg)

                if (isNaN(decdeg)) throw "Bad Input lat DD"

                lat = decdeg
            } else if ($("#antenna-lat-fieldset").attr('format') == 'DDM') {
                let dec = parseFloat($('#antenna-lat-DDM-deg-input').val())
                let min = parseFloat($('#antenna-lat-DDM-dec-min-input').val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"

                $("#antenna-lat-DDM-deg-input").attr("value", dec)
                $("#antenna-lat-DDM-dec-min-input").attr("value", min)

                lat = dec + (min / 60)
            } else if ($("#antenna-lat-fieldset").attr('format') == 'DMS') {
                let dec = parseFloat($('#antenna-lat-DMS-deg-input').val())
                let mins = parseFloat($('#antenna-lat-DMS-min-input').val())
                let secs = parseFloat($('#antenna-lat-DMS-sec-input').val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS"

                $("#antenna-lat-DMS-deg-input").attr("value", dec)
                $("#antenna-lat-DMS-min-input").attr("value", mins)
                $("#antenna-lat-DMS-sec-input").attr("value", secs)

                lat = dec + (mins / 60 + secs / 3600)
            }

            if ($("#antenna-long-fieldset").attr('format') == 'DD') {
                let decdeg = parseFloat(($("#antenna-long-DD-dec-deg-input").val()))

                if (isNaN(decdeg)) throw "Bad Input long DD"

                $("#antenna-long-DD-dec-deg-input").attr("value", decdeg)

                long = decdeg
            } else if ($("#antenna-long-fieldset").attr('format') == 'DDM') {
                let dec = parseFloat($('#antenna-long-DDM-deg-input').val())
                let min = parseFloat($('#antenna-long-DDM-dec-min-input').val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input long DDM"

                $("#antenna-long-DDM-deg-input").attr("value", dec)
                $("#antenna-long-DDM-dec-min-input").attr("value", min)

                long = dec + (min / 60)
            } else if ($("#antenna-long-fieldset").attr('format') == 'DMS') {
                let dec = parseFloat($('#antenna-long-DMS-deg-input').val())
                let mins = parseFloat($('#antenna-long-DMS-min-input').val())
                let secs = parseFloat($('#antenna-long-DMS-sec-input').val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS"

                $("#antenna-long-DMS-deg-input").attr("value", dec)
                $("#antenna-long-DMS-min-input").attr("value", mins)
                $("#antenna-long-DMS-sec-input").attr("value", secs)

                long = dec + (mins / 60 + secs / 3600)
            }
            let bearing = parseFloat($("#antenna-bearing-input").val())
            if (isNaN(bearing)) throw "Bad input bearing"

            $("#antenna-bearing-input").attr("value", bearing)


            $('.antenna-input-field').prop('disabled', true)
            $('.antenna-change-btn').prop('disabled', false)
            $('#antenna-confirm-btn').prop('disabled', true)

            createAntennaBearingChangeButtonHandler()

            //got_antenna_pos.set(false)

            //ROS params
            antenna_latitude.set(lat)
            antenna_longitude.set(long)
            antenna_start_dir.set(bearing)
            $('#antenna-stats-lat').text(lat.toFixed(6))
            $('#antenna-stats-long').text(long.toFixed(6))
            $('#antenna-stats-heading').text(bearing)
            appendToConsole('Antenna parmameters have been set!')
        } catch (e) {
            appendToConsole(e)
        }
    })

    function antennaChangeButtonHandlerTemplate(mode, format, detachedData) {
        $("#antenna-" + mode + "-" + format + "-change-btn").on('click', function(event) {
            event.preventDefault()

            $("#antenna-" + mode + "-input-group").detach()
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

            $.ajax('/navigation/inputTemplates/antenna-' + format + '/' + mode, {
                success: function(result) {
                    $("#antenna-" + mode + "-fieldset").append(result)

                    if (mode == 'lat') createAntennaLatitudeChangeButtonHandler(detachedData)
                    else createAntennaLongitudeChangeButtonHandler(detachedData)
                }
            })
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


    $("#new-goal-coordinates-btn").on('click', function(event) {
        // this call appends the inputTemplate to the goal-modal-body-content
        // which we can keep, just chnage the count situation to use the list
        $.ajax('/navigation/inputTemplates/new-goal-coordinates-btn/' + count, {
            success: function(result) {
                $("#goal-modal-body-content").append(result)

                createGoalLatitudeHandler(count)
                createGoalLongitudeHandler(count)
                createGoalButtons(count)
                count++
            }
        })
    })

    function insertDataInQueue(current, lat_format, long_format) {
        // the queue doesn't exist anymore you fuck
        // just send the data to the node
        // maybe rename this function into confirmGoalData or something
        let lat = null
        let long = null
        try {
            if (lat_format == 0) {
                //decimal-degrees mode
                decdeg = parseFloat($('#goal-lat-DD-dec-deg-input-' + current).val())

                if (isNaN(decdeg)) throw "Bad input lat DD"

                $('#goal-lat-DD-dec-deg-input-' + current).attr("value", decdeg)

                lat = decdeg
            } else if (lat_format == 1) {
                //degrees-decimal-minutes mode
                let dec = parseFloat($('#goal-lat-DDM-deg-input-' + current).val())
                let min = parseFloat($('#goal-lat-DDM-dec-min-input-' + current).val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"

                lat = dec + (min / 60)

                $('#goal-lat-DDM-deg-input-' + current).attr("value", dec)
                $('#goal-lat-DDM-dec-min-input-' + current).attr("value", min)

            } else if (lat_format == 2) {
                //degress-minutes-seconds mode
                let dec = parseFloat($('#goal-lat-DMS-deg-input-' + current).val())
                let mins = parseFloat($('#goal-lat-DMS-min-input-' + current).val())
                let secs = parseFloat($('#goal-lat-DMS-sec-input-' + current).val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS"

                lat = dec + (mins / 60 + secs / 3600)

                $('#goal-lat-DMS-deg-input-' + current).attr("value", dec)
                $('#goal-lat-DMS-min-input-' + current).attr("value", mins)
                $('#goal-lat-DMS-sec-input-' + current).attr("value", secs)
            } else {
                throw "You didn't enter anything for lat"
            }
            if (long_format == 0) {
                //decimal-degrees mode
                long = parseFloat($('#goal-long-DD-dec-deg-input-' + current).val())

                if (isNaN(long)) throw "Bad input long DD"

                $('#goal-long-DD-dec-deg-input-' + current).attr("value", long)
            } else if (long_format == 1) {
                //degrees-decimal-minutes mode
                let dec = parseFloat($('#goal-long-DDM-deg-input-' + current).val())
                let min = parseFloat($('#goal-long-DDM-dec-min-input-' + current).val())

                if (isNaN(dec) || isNaN(min)) throw "Bad input long DDM"

                long = dec + (min / 60)

                $('#goal-long-DDM-deg-input-' + current).attr("value", dec)
                $('#goal-long-DDM-dec-min-input-' + current).attr("value", min)

            } else if (long_format == 2) {
                //degress-minutes-seconds mode
                let dec = parseFloat($('#goal-long-DMS-deg-input-' + current).val())
                let mins = parseFloat($('#goal-long-DMS-min-input-' + current).val())
                let secs = parseFloat($('#goal-long-DMS-sec-input-' + current).val())

                if (isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS"

                long = dec + (mins / 60 + secs / 3600)

                $('#goal-long-DMS-deg-input-' + current).attr("value", dec)
                $('#goal-long-DMS-min-input-' + current).attr("value", mins)
                $('#goal-long-DMS-sec-input-' + current).attr("value", secs)
            } else {
                throw "You didn't enter anything for long"
            }

            goalName = $('#goal-name-' + current).val()
            if (goalName.length != 0) $('#goal-name-' + current).attr('value', goalName)
            else $('#goal-name-' + current).attr('value', 'Goal-' + current)

            $('#goal-name-fieldset-' + current).prop('disabled', true)
            $('#goal-lat-fieldset-' + current).prop('disabled', true)
            $('#goal-long-fieldset-' + current).prop('disabled', true)

            let latlongpair = {
                lat: lat,
                long: long
            }

            // maybe put the rest of this in a separate function

            $("#goal-modal-body-content").attr("count", count)


            //the confirm button's functionality will change depending on wether or not the change button was clicked before it.
            if ($("#goal-change-btn-" + current).data('clicked')) {
                $(this).data('clicked', false)
                // maybe make a change goal option here
            } else {
                // add the goal to data
            }

            //change stats to next goal
            $('#goal-stats-lat').text()
            $('#goal-stats-long').text()


            /*----------------------------------\
            this section implements ROS goal node
            \----------------------------------*/
            let rosGoalName = $('#goal-name-' + current).val()
            let rosGoalLong = long
            let rosGoalLat = lat
            let goalData = new ROSLIB.Message({
                name: rosGoalName,
                long: rosGoalLong,
                lat: rosGoalLat
            })

            create_goal_publisher.publish(goalData)

        } catch (e) {
            $('#goal-confirm-btn-' + current).prop('disabled', false)
            $('#goal-change-btn-' + current).prop('disabled', true)

            appendToConsole(e)
        }
    }

    function goalHandlerTemplate(mode, format, current) {
        $("#goal-" + mode + "-" + format + "-btn-" + current).on('click', function(event) {
            event.preventDefault()

            $('#goal-' + mode + '-fieldset-' + current).attr("format", format)
            $('#goal-' + mode + '-select-format-btn-' + current).dropdown('toggle')
            $('#goal-' + mode + '-select-format-btn-' + current).detach()

            $.ajax('/navigation/inputTemplates/goal-' + format + '/' + mode + '/' + current, {
                success: function(result) {
                    $('#goal-' + mode + '-input-group-' + current).append(result)
                }
            })
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
        $.ajax('navigation/inputTemplates/goal-buttons/' + current, {
            success: function(result) {
                $("#goal-buttons-input-group-" + current).append(result)

                createGoalConfirmButtonHandler(current)
                createGoalChangeButtonHandler(current)
                createGoalDeleteButtonHandler(current)
            }
        })
    }

    function createGoalConfirmButtonHandler(current) {
        $('#goal-confirm-btn-' + current).on('click', function(event) {
            let lat_format = -1
            let long_format = -1

            if ($("#goal-lat-fieldset-" + current).attr("format") == 'DD')
                lat_format = 0
            else if ($("#goal-lat-fieldset-" + current).attr("format") == 'DDM')
                lat_format = 1
            else if ($("#goal-lat-fieldset-" + current).attr("format") == 'DMS')
                lat_format = 2
            else console.log('error with goal lat')

            if ($("#goal-long-fieldset-" + current).attr("format") == 'DD')
                long_format = 0
            else if ($("#goal-long-fieldset-" + current).attr("format") == 'DDM')
                long_format = 1
            else if ($("#goal-long-fieldset-" + current).attr("format") == 'DMS')
                long_format = 2
            else console.log('error with goal long')

            $('#goal-change-btn-' + current).prop('disabled', false)
            $('#goal-confirm-btn-' + current).prop('disabled', true)

            $('#goal-lat-select-format-' + current).detach()
            $('#goal-long-select-format-' + current).detach()

            insertDataInQueue(current, lat_format, long_format)
        })
    }

    function createGoalChangeButtonHandler(current) {
        $('#goal-change-btn-' + current).on('click', function(event) {
            $(this).data('clicked', true)

            $('#goal-lat-fieldset-' + current).prop('disabled', false)
            $('#goal-long-fieldset-' + current).prop('disabled', false)

            $('#goal-change-btn-' + current).prop('disabled', true)
            $('#goal-confirm-btn-' + current).prop('disabled', false)
        })
    }

    function createGoalDeleteButtonHandler(current) {
        $('#goal-delete-btn-' + current).on('click', function(event) {

            /*------------------------\
            implement ROS goal deleting
            \------------------------*/
            let goalName = $('#goal-name-' + current).val()
            let msg = new ROSLIB.Message({
                data: goalName
            })
            delete_goal_publisher.publish(msg)


            $('#new-goal-btn-' + current).remove()

            // set new stats to the new goal and update params..
            if (!(navQueue.top() == undefined)) {
                $('#goal-stats-lat').text(navQueue.top().lat.toFixed(6))
                $('#goal-stats-long').text(navQueue.top().long.toFixed(6))
            } else {
                $('#goal-stats-lat').text("----")
                $('#goal-stats-long').text("----")
                appendToConsole('No coordinates left to go to!')
            }
        })
    }
})
