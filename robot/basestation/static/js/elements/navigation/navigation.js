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
    let goal_publisher = new ROSLIB.Topic({
        ros: ros,
        name: 'create_goal',
        messageType: 'mcu_control/RoverGoal'
    })
    let goal_list_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: 'goal_list',
        messageType: 'mcu_control/RoverGoalList'
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
    let got_antenna_pos = new ROSLIB.Param({
        ros: ros,
        name: 'got_antenna_pos'
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

            //remove the most recent coordinate pair from the GUI
            for (i = 0; i < count; i++) {

                if ($('#new-goal-btn-' + i).length) {
                    $('#new-goal-btn-' + i).remove()
                    break
                }
            }


            navQueue.data[0] = null
            //the elements are deleted from data, they are only set to null. The array will only get resized in this case, where those null values will be destroyed
            while (navQueue.top() == null && (navQueue.data.length != 0)) {
                navQueue.dequeue()
            }
            saveGoalModalToServer()
            saveNavigationQueueToServer()


            if (navQueue.top() != undefined) {
                goal_latitude.set(parseFloat(navQueue.top().latitude))
                goal_longitude.set(parseFloat(navQueue.top().longitude))

                $('#goal-stats-lat').text(navQueue.top().latitude.toFixed(6))
                $('#goal-stats-long').text(navQueue.top().longitude.toFixed(6))

                appendToConsole('Moving to next coordinate!')

            } else {
                $('#goal-stats-lat').text('----')
                $('#goal-stats-long').text('----')
            }

            has_gps_goal.set(false)

        }
        $('#goal-stats-rover-heading').text(parseFloat(message.x).toFixed(3))
        $('#goal-stats-distance').text(parseFloat(message.y).toFixed(2))
        saveGoalStatsToServer()

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

    let navQueue = new navigationQueue()
    let count = 0

    function initNavigationPanel() {

        antenna_latitude.get(function(lat) {
            if (lat != null) {
                antenna_longitude.get(function(long) {
                    if (long != null) {
                        antenna_start_dir.get(function(dir) {
                            if (dir != null) {
                                appendToConsole('Antenna parameters already set')
                                got_antenna_pos.set(true)

                                $.get({
                                    url: '/navigation/cached_content/antenna_stats',
                                    success: function(result) {
                                        $('#antenna-stats-lat').text(lat.toFixed(6))
                                        $('#antenna-stats-long').text(long.toFixed(6))
                                        $('#antenna-stats-heading').text(dir)

                                        if (!jQuery.isEmptyObject(result)) {
                                            $('#antenna-stats-dist-to-rover').text(JSON.parse(result).distance)
                                            $('#antenna-stats-rec-angle').text(JSON.parse(result).heading)
                                        } else {
                                            $('#antenna-stats-dist-to-rover').text("----")
                                            $('#antenna-stats-rec-angle').text("----")
                                        }
                                    },

                                })
                                $.get({
                                    url: '/navigation/cached_content/antenna_modal',
                                    success: function(result) {

                                        let detachedLatFormatData = $('#antenna-select-lat-format').detach()
                                        let detachedLongFormatData = $('#antenna-select-long-format').detach()

                                        $('#antenna-modal-body').empty()
                                        $('#antenna-modal-body').append(result)

                                        createAntennaLatitudeChangeButtonHandler(detachedLatFormatData)
                                        createAntennaLongitudeChangeButtonHandler(detachedLongFormatData)
                                        createAntennaBearingChangeButtonHandler()
                                    }
                                })
                            } else {
                                appendToConsole('Enter antenna parameters')
                                got_antenna_pos.set(false)

                            }
                        })

                    } else {
                        appendToConsole('Enter antenna parameters')
                        got_antenna_pos.set(false)

                    }
                })
            } else {
                appendToConsole('Enter antenna parameters')
                got_antenna_pos.set(false)
            }
        })

        goal_latitude.get(function(lat) {
            if (lat != null) {
                goal_longitude.get(function(long) {
                    if (long != null) {
                        appendToConsole('Goal coordinates already set')
                        has_gps_goal.set(true)
                        $.get({
                            url: '/navigation/cached_content/goal_modal',
                            success: function(result) {

                                $('#goal-modal-body').empty()
                                $('#goal-modal-body').append(result)

                                //this variable keeps track of the number of things created, so the event handlers can properly be re-initiazed.
                                count = $('#goal-modal-body-content').attr('count')

                                for (i = 0; i < count; i++) {
                                    if ($('#new-goal-btn-' + i).length) {
                                        createGoalLatitudeHandler(i)
                                        createGoalLongitudeHandler(i)
                                        createGoalChangeButtonHandler(i)
                                        createGoalDeleteButtonHandler(i)
                                        createGoalConfirmButtonHandler(i)
                                    }
                                }

                            }
                        })
                        $.get({
                            url: '/navigation/cached_content/navQueue',
                            success: function(result) {

                                if (!jQuery.isEmptyObject(result)) {
                                    navQueue.data = JSON.parse(result).data
                                    navQueue.shift = JSON.parse(result).shift
                                    if (navQueue.data.length != 0)
                                        navQueue.flag = false
                                }
                            }
                        })
                        $.get({
                            url: '/navigation/cached_content/goal_stats',
                            success: function(result) {

                                //the first two are saved as ROS params, while the bottom two saved on the server, hence the difference
                                $('#goal-stats-lat').text(lat.toFixed(6))
                                $('#goal-stats-long').text(long.toFixed(6))

                                if (!jQuery.isEmptyObject(result)) {
                                    $("#goal-stats-rover-heading").text(JSON.parse(result).heading)
                                    $("#goal-stats-distance").text(JSON.parse(result).distance)
                                } else {
                                    $("#goal-stats-rover-heading").text('----')
                                    $("#goal-stats-distance").text('----')
                                }
                            }
                        })
                    } else {
                        appendToConsole("Enter goal parmameters")
                    }
                })
            } else {
                appendToConsole("Enter goal paramaters")
            }
        })
    }
    //theres probably a better place to put this, this is just a temporary fix
    initNavigationPanel()

    // 0 1 2 3 4 5
    function navigationQueue() {
        this.data = []
        this.shift = 0
        this.flag = true

        this.enqueue = function(item) {

            //the first time something is added to the queue, it should be set as current goal paramaters, so this code should only happen once
            if (this.flag || this.data.length == 0) {
                has_gps_goal.set(false)
                goal_latitude.set(item.latitude)
                goal_longitude.set(item.longitude)
                this.flag = false
            }
            this.data.push(item)

        }
        this.dequeue = function() {
            this.data.shift()
            this.shift++
        }
        this.top = function() {
            return this.data[0]
        }
        this.remove = function(index) {
            this.data[index - this.shift] = null

            //if the current top is deleted, gotta adjust the goal_lat and goal_long parmamters
            if (index - this.shift == 0) {

                while (navQueue.top() == null) {
                    navQueue.dequeue()
                    if (navQueue.data.length == 0) {
                        break
                    }
                }
                if (navQueue.data.length != 0) {
                    goal_latitude.set(this.top().latitude)
                    goal_longitude.set(this.top().longitude)
                    has_gps_goal.set(false)
                }

            }
        }

        this.change = function(index, lat, long) {
            this.data[index - this.shift].latitude = lat
            this.data[index - this.shift].longitude = long

            if ((index - this.shift) == 0) {
                goal_latitude.set(lat)
                goal_longitude.set(long)
                has_gps_goal.set(false)
            }
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
            //ROS params
            antenna_latitude.set(lat)
            antenna_longitude.set(long)
            antenna_start_dir.set(bearing)

            appendToConsole('antenna parmameters have been set!')

            $('.antenna-input-field').prop('disabled', true)
            $('.antenna-change-btn').prop('disabled', false)

            $('#antenna-confirm-btn').prop('disabled', true)
            createAntennaBearingChangeButtonHandler()


            got_antenna_pos.set(false)

            $('#antenna-stats-lat').text(lat.toFixed(6))
            $('#antenna-stats-long').text(long.toFixed(6))
            $('#antenna-stats-heading').text(bearing)

            saveAntennaModalToServer()

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

            $('#goal-name-fieldset-' + current).prop('disabled', true)
            $('#goal-lat-fieldset-' + current).prop('disabled', true)
            $('#goal-long-fieldset-' + current).prop('disabled', true)

            let latlongpair = {
                latitude: lat,
                longitude: long
            }

            $("#goal-modal-body-content").attr("count", count)

            saveGoalModalToServer()


            //the confirm button's functionality will change depending on wether or not the change button was clicked before it.
            if ($("#goal-change-btn-" + current).data('clicked')) {

                $(this).data('clicked', false)
                navQueue.change(current, lat, long)
            } else {
                if (navQueue.data.length == 0) appendToConsole('Goal data set!')
                navQueue.enqueue(latlongpair)

            }

            $('#goal-stats-lat').text(navQueue.top().latitude.toFixed(6))
            $('#goal-stats-long').text(navQueue.top().longitude.toFixed(6))

            saveNavigationQueueToServer()





            /*----------------------------------\
            this section implements ROS goal node
            \----------------------------------*/
            let rosGoalName = $('#goal-name-' + current).val()
            let rosGoalLong = long
            let rosGoalLat = lat

            let goal_data = new ROSLIB.Message({
                name : rosGoalName,
                long : rosGoalLong,
                lat : rosGoalLat
            })

            goal_publisher.publish(goal_data)







        } catch (e) {
            $('#goal-confirm-btn-' + current).prop('disabled', false)
            $('#goal-change-btn-' + current).prop('disabled', true)
            appendToConsole(e)
        }
        console.log(navQueue)
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


    function saveNavigationQueueToServer() {
        $.post({
            url: "/navigation/cached_content/navQueue",
            contentType: "application/json",
            data: JSON.stringify(navQueue)
        })
    }

    function saveGoalModalToServer() {
        $.post({
            url: "/navigation/cached_content/goal_modal",
            data: $('#goal-modal-body').html()
        })

    }

    function saveAntennaModalToServer() {
        $.post({
            url: "/navigation/cached_content/antenna_modal",
            data: $('#antenna-modal-body').html(),

        })
    }

    function saveGoalStatsToServer() {
        let obj = {
            distance: $("#goal-stats-distance").text(),
            heading: $("#goal-stats-rover-heading").text()

        }
        $.post({
            url: "/navigation/cached_content/goal_stats",
            contentType: "application/json",
            data: JSON.stringify(obj)
        })
    }

    function saveAntennaStatsToServer() {
        let obj = {
            distance: $("#antenna-stats-dist-to-rover").text(),
            heading: $("#antenna-stats-rec-angle").text()

        }
        $.post({
            url: "/navigation/cached_content/antenna_stats",
            contentType: "application/json",
            data: JSON.stringify(obj)
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
            $('#new-goal-btn-' + current).remove()

            navQueue.remove(current)

            if (!(navQueue.top() == undefined)) {

                $('#goal-stats-lat').text(navQueue.top().latitude.toFixed(6))
                $('#goal-stats-long').text(navQueue.top().longitude.toFixed(6))
            } else {
                $('#goal-stats-lat').text("----")
                $('#goal-stats-long').text("----")
                appendToConsole('No coordinates left to go to!')
            }
            saveGoalModalToServer()
            saveNavigationQueueToServer()

        })
    }

    initNavigationPanel()
})
