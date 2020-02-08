$(document).ready(() => {
  const send_cmd = $('#send-serial-cmd')
  const cmd_input = $('#serial-cmd-input')
  const INVALID_CMD_MSG = 'invalid command, please check input and try again'
  const INVALID_MODULE_MSG = 'Invalid module, please input module before command. Valid modules are rover, arm, IK and PDS'

  function getCMD() {
    const full_cmd = cmd_input.val()
    const prefix = full_cmd.split(' ').shift()
    const cmd = full_cmd.split(' ').pop()
    if (isCMDValid(full_cmd, cmd)) {
      sendCMD(prefix, cmd)
    }
    clearCmdInput()
  }

  function isCMDValid(full_cmd, cmd) {
    if ((cmd == '' && full_cmd != '') || (cmd == full_cmd)) {
      if (full_cmd != '') {
        appendToConsole(INVALID_CMD_MSG)
      }
      return false
    } else {
      return true
    }
  }

  function sendCMD(prefix, cmd) {
    switch(prefix) {
      case 'rover':
        sendRoverCommand(cmd)
        break
      case 'arm':
        sendArmCommand(cmd)
        break
      case 'IK':
        sendIKCommand(cmd)
        break
      case 'PDS':
        sendPdsCommand(cmd)
        break
      default:
        appendToConsole(INVALID_MODULE_MSG)
    }
  }

  function clearCmdInput () {
    $(cmd_input).val('')
  }

  send_cmd.click(getCMD)

  document.getElementById('serial-cmd-input').addEventListener('keyup', function (event) {
    if (event.code === 'Enter') {
      getCMD()
    }
  })
})
