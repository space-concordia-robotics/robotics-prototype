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
    if(degrees > SERVO_ANGLE_MIN && degrees < SERVO_ANGLE_MAX)
      sendRoverCommand(getPanSymbol(cameraName) + degrees);
  }

  function cameraTilt(cameraName, degrees)
  {
    if(degrees > SERVO_ANGLE_MIN && degrees < SERVO_ANGLE_MAX)
      sendRoverCommand(getTiltSymbol(cameraName) + degrees);
  }

})
