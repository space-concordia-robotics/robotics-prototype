$(document).ready(() => {
  const send_cmd = $('#send-serial-cmd')
  const cmd_input = $('#serial-cmd-input')
  const invalid_cmd_msg = 'invalid command, please check input and try again'
  const invalid_module_msg = 'Invalid module, please input module before command. Valid modules are rover, arm, ik and pds'

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
    if ((cmd == '' && full_cmd != '') || cmd == full_cmd) {
      appendToConsole(invalid_cmd_msg)
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
      case 'ik':
        sendIKCommand(cmd)
        break
      case 'pds':
        sendPdsCommand(cmd)
        break
      default:
        appendToConsole(invalid_module_msg)
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
