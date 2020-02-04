$(document).ready(() => {
  const FRONT_CAM_NAME = 'videoFrontCam'
  const BACK_CAM_NAME = 'videoRearCam'
  const FRONT_PAN_SYM = '!'
  const FRONT_TILT_SYM = '@'
  const BACK_PAN_SYM = '#'
  const BACK_TILT_SYM = '$'
  const SERVO_SYMBOLS = {}
  SERVO_SYMBOLS[FRONT_CAM_NAME] = {
      'pan': FRONT_PAN_SYM,
      'tilt': FRONT_TILT_SYM
    }
  SERVO_SYMBOLS[BACK_CAM_NAME] = {
      'pan': BACK_PAN_SYM,
      'tilt': BACK_TILT_SYM
  } 

  const SERVO_ANGLE_MIN = 0;
  const SERVO_ANGLE_MAX = 180;
  const SERVO_INCREMENT = 5;

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

  function cameraPan(cameraName, degrees)
  {
      sendRoverCommand(getPanSymbol(cameraName) + degrees);
  }

  function cameraTilt(cameraName, degrees)
  {
      sendRoverCommand(getTiltSymbol(cameraName) + degrees);
  }

  $('.camera-tilt-up').click(e => {
    let cameraControls = $(e.target).parents('.camera-controls')
    let cameraName = cameraControls.attr('camera-name');

    if(hasCameraControls(cameraName))
    {
      let tilt = cameraControls.attr('tilt');
      tilt = clamp(parseInt(tilt) + SERVO_INCREMENT, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
      cameraTilt(cameraName, tilt);
      cameraControls.attr('tilt', toString(tilt));
    }

  })

  $('.camera-tilt-down').click(e => {
    let cameraControls = $(e.target).parents('.camera-controls')
    let cameraName = cameraControls.attr('camera-name');

    if(hasCameraControls(cameraName))
    {
      let tilt = cameraControls.attr('tilt');
      tilt = clamp(parseInt(tilt) - SERVO_INCREMENT, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
      cameraTilt(cameraName, tilt);
      cameraControls.attr('tilt', toString(tilt));
    }
  })

  $('.camera-pan-left').click(e => {
    let cameraControls = $(e.target).parents('.camera-controls')
    let cameraName = cameraControls.attr('camera-name');

    if(hasCameraControls(cameraName))
    {
      let pan = cameraControls.attr('pan');
      pan = clamp(parseInt(pan) - SERVO_INCREMENT, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
      cameraTilt(cameraName, pan);
      cameraControls.attr('pan', toString(pan));
    }
  })

  $('.camera-pan-right').click(e => {
    let cameraControls = $(e.target).parents('.camera-controls')
    let cameraName = cameraControls.attr('camera-name');

    if(hasCameraControls(cameraName))
    {
      let pan = cameraControls.attr('pan');
      pan = clamp(parseInt(pan) + SERVO_INCREMENT, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
      cameraTilt(cameraName, pan);
      cameraControls.attr('pan', toString(pan));
    }
  })
})
