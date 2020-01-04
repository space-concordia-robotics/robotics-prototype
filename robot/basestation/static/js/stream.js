$(document).ready(function () {
  const STREAM_OFF = '../static/images/stream-offline.jpg'

  $('#arm-stream-btn').on('click', function (event) {
    if ($('#arm-stream-btn').is(':checked')) {
      // NOT WORKING YET BECAUSE NOT CALLING THE TASK HANDLER AFTER "check" is made
      // see cameras.js for example
      $('img#arm-camera-feed')[0].src = getStreamURL('ArmScienceCam')
    } else {
      $('img#arm-camera-feed')[0].src = STREAM_OFF
    }
  })
  $('#front-stream-btn').on('click', function (event) {
    if ($('#front-stream-btn').is(':checked')) {
      $('img#front-camera-feed')[0].src = getStreamURL('FrontCam')
    } else {
      $('img#front-camera-feed')[0].src = STREAM_OFF
    }
  })

  $('#rear-stream-btn').on('click', function (event) {
    if ($('#rear-stream-btn').is(':checked')) {
      $('img#rear-camera-feed')[0].src = getStreamURL('RearCam')
    } else {
      $('img#rear-camera-feed')[0].src = STREAM_OFF
    }
  })
})
