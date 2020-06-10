$(document).ready(() => {
  const sendCommand = $('#send-serial-command')
  const commandInput = $('#serial-command-input')
  const INVALID_COMMAND_MSG = 'Invalid command, please check input and try again'
  const INVALID_MODULE_MSG = 'Invalid module, please input module before command. Valid modules are rover, arm, science and pds'

  // Get full command and split into its prefix and command to send to module
  function processCommand() {
    const fullCommand = commandInput.val()
    const prefix = fullCommand.split(' ').shift()
    const command = fullCommand.split(prefix + ' ').pop()
    if (isCommandValid(fullCommand, command)) {
      sendInputCommand(prefix.toLowerCase(), fullCommand)
    }
  }

  // Check if the command is valid
  function isCommandValid(fullCommand, command) {
    if ((command == '' && fullCommand != '') || (command == fullCommand)) {
      if (fullCommand != '') {
        logErr(INVALID_COMMAND_MSG)
      }
      return false
    }
    return true
  }

  // Send command to module, print error if invalid module was inputted
  function sendInputCommand(prefix, command) {
    switch(prefix) {
      case 'rover':
        sendRoverCommand(command)
        clearCommandInput()
        break
      case 'arm':
        sendArmCommand(command)
        clearCommandInput()
        break
      case 'pds':
        sendPdsCommand(command)
        clearCommandInput()
        break
      case 'science':
        sendScienceCommand(command)
        clearCommandInput()
        break
      default:
        logErr(INVALID_MODULE_MSG)
    }
  }

  // Clear command input field
  function clearCommandInput () {
    $(commandInput).val('')
  }

  // Run getCMD in order to process command inputted in field, when send command button is clicked
  sendCommand.click(processCommand)

  // Run getCMD in order to process command inputted into box, when enter key is pressed while focused on input field
  document.getElementById('serial-command-input').addEventListener('keyup', function (event) {
    if (event.code === 'Enter') {
      processCommand()
    }
  })
})
