$(document).ready(() => {
  const send_cmd = $('#send-serial-cmd')
  const cmd_input = $('#serial-cmd-input')

  function handleCMD() {
    const full_cmd = cmd_input.val()
    const prefix = full_cmd.split(' ').shift()
    const cmd = full_cmd.split(' ').pop()
    if (cmd != '' && cmd != full_cmd) {
      sendCMD(prefix, cmd)
    }

    else {
      appendToConsole('no command to send')
    }
  }

  function sendCMD(prefix, cmd) {
    if (prefix == 'rover') {
      sendRoverCommand(cmd)
    } else if (prefix == 'arm') {
      sendArmCommand(cmd)
    } else if (prefix == 'ik') {
      sendIKCommand(cmd)
    } else if (prefix == 'pds') {
      sendPdsCommand(cmd)
    } else {
      appendToConsole(prefix + 'is not a module')
    }
    clearCmdInput()
  }

  function clearCmdInput () {
    $(cmd_input).val('')
  }

  send_cmd.click(function () {
    handleCMD()
  })

  document.getElementById('serial-cmd-input').addEventListener('keyup', function (event) {
    if (event.code === 'Enter') {
      handleCMD()
    }
  })
})
