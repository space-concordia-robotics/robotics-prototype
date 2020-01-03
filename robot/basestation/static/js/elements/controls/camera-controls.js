$(document).ready(() => {

  const FRONT_CAM_NAME = ''
  const BACK_CAM_NAME = ''
  const FRONT_PAN_SYM = '!'
  const FRONT_TILT_SYM = '@'
  const BACK_PAN_SYM = '#'
  const BACK_TILT_SYM = '$'
  const SERVO_SYMBOLS = { 
    FRONT_CAM_NAME : {
        'pan': FRONT_PAN_SYM,
        'tilt': FRONT_TILT_SYM
      },
    BACK_CAM_NAME : {
      'pan': BACK_PAN_SYM,
      'tilt' : BACK_TILT_SYM
      }
  }

  function hasCameraControls(cameraName)
  {
    return cameraName == FRONT_CAM_NAME || cameraName == BACK_CAM_NAME
  }

  function getPanSymbol(cameraName)
  {
    return SERVO_SYMBOLS[cameraName]['pan']
  }

  function getTiltSymbol(cameraName)
  {
    return SERVO_SYMBOLS[cameraName]['tilt']
  }
  
  $('#camera-front-lpan-btn').click(function () {
      sendRoverCommand(
        '!' + $('#servo-val').val()
      )
  })
  $('#camera-front-rpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('!' + $('#servo-val').val())
    }
  })
  // servo name: "Front camera Side continuous servo"
  $('#camera-front-tilt-up-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('@' + $('#servo-val').val())
    }
  })
  $('#camera-front-tilt-down-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('@' + $('#servo-val').val())
    }
  })
  // servo name: "Rear camera positional tilt base"
  $('#camera-back-lpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('#' + $('#servo-val').val())
    }
  })
  $('#camera-back-rpan-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('#' + $('#servo-val').val())
    }
  })
  // servo name: "Rear camera Side continuous servo"
  $('#camera-back-tilt-up-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('$' + $('#servo-val').val())
    }
  })

  $('#camera-back-tilt-down-btn').click(function () {
    if ($('#servo-val').val() != '') {
      sendRoverCommand('$' + $('#servo-val').val())
    }
  })
})
