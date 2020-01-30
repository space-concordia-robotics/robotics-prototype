$(document).ready(() => {
  const send_cmd = $('#send-serial-cmd')
  const cmd_input = $('#serial-cmd-input')

  function sendCMD() {
    const full_cmd = cmd_input.val()
    const prefix = full_cmd.split(' ').shift()
    const cmd = full_cmd.split(' ').pop()
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

  send_cmd.click(function () {
    sendCMD()
  })

  document.getElementById('serial-cmd-input').addEventListener('keyup', function (event) {
    if (event.code === 'Enter') {
      sendCMD()
    }
  })

  function clearCmdInput () {
    $(cmd_input).val('')
  }
})
