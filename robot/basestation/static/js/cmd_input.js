$(document).ready(() => {
  function sendCMD() {
    const full_cmd = $('#serial-cmd-input').val()
    const target = full_cmd.split(' ').shift()
    const cmd = full_cmd.split(' ').pop()
    if (target == 'rover') {
      sendRoverCommand(cmd)
    } else if (target == 'arm') {
      sendArmCommand(cmd)
    }
  }

  $('#send-serial-cmd').click(function () {
    sendCMD()
  })

  document.addEventListener('keyup', function (event) {
    if ($('#serial-cmd-input').is(':focus') && event.code === 'Enter') {
      sendCMD()
    }
  })
})
