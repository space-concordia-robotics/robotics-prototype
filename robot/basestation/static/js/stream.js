$(document).ready(function () {
    $('#arm-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#arm-stream-btn').is(':checked')) {
            $('img#arm-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/ArmCamera/image_raw'
        } else {
            $('img#arm-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })
    $('#front-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#front-stream-btn').is(':checked')) {
            $('img#front-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/FrontCamera/image_raw'
        } else {
            $('img#front-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })

    $('#rear-stream-btn').on('click', function (event) {
        // click makes it checked during this time, so trying to enable
        if ($('#rear-stream-btn').is(':checked')) {
            $('img#rear-camera-feed')[0].src =
            'http://localhost:8080/stream?topic=/RearCamera/image_raw'
        } else {
            $('img#rear-camera-feed')[0].src =
                '../static/images/stream-offline.jpg'
        }
    })
})
