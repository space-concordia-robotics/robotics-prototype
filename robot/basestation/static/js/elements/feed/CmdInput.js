$(document).ready(() => {
  const send_cmd = $('#send-serial-cmd')
  const cmd_input = $('#serial-cmd-input')
  const INVALID_CMD_MSG = 'invalid command, please check input and try again'
  const INVALID_MODULE_MSG = 'Invalid module, please input module before command. Valid modules are rover, arm, IK and PDS'

  // Get full command and split into its prefix and command to send to module
  function getCMD() {
    const full_cmd = cmd_input.val()
    const prefix = full_cmd.split(' ').shift()
    const cmd = full_cmd.split(prefix + ' ').pop()
    console.log(prefix)
    if (isCMDValid(full_cmd, cmd)) {
      sendCMD(prefix.toLowerCase(), cmd)
    }
    clearCmdInput()
  }

  // Check if the command is valid
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

  // Send command to module, print error if invalid module was inputted
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
        appendToConsole(INVALID_MODULE_MSG)
    }
  }

  // Clear command input field
  function clearCmdInput () {
    $(cmd_input).val('')
  }

  // Run getCMD in order to process command inputted in field, when send command button is clicked
  send_cmd.click(getCMD)

  // Run getCMD in order to process command inputted into box, when enter key is pressed while focused on input field
  document.getElementById('serial-cmd-input').addEventListener('keyup', function (event) {
    if (event.code === 'Enter') {
      getCMD()
    }
  })
})
