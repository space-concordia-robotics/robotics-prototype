$('#homing-button').on('click', function (event) {
  event.preventDefault()
  // TODO: uncomment this when homing is confirmed to work properly
  // sendArmCommand('home') // REIMPLEMENT AS AN ACTION
})

$('#list-all-cmds').on('click', function (event) {
  event.preventDefault()
  printCommandsList()
})

// TODO: This scheme of calling both arm and rover should be revised. I suggest to split extracontrols and pass a parameter
$('#stop-all-motors').on('click', function (event) {
  event.preventDefault()
  sendArmCommand('stop')
  sendRoverCommand('stop')
})

$('#reset-motor-angles').on('click', function (event) {
  event.preventDefault()
  sendArmCommand('reset')
  sendRoverCommand('reset')
})

$('#reboot-button').on('click', function (event) {
  event.preventDefault()
  sendArmCommand('reboot')
  sendRoverCommand('stop')
})

$('#flip-stream').on('click', function () {
  $('#camera-feed').toggleClass('rotateimg180')
})

$('#flip-stream-ccw').on('click', function () {
  $('#camera-feed').toggleClass('rotateimgccw')
  $('#camera-feed').toggleClass('stretch-down')
})

$('#flip-stream-cw').on('click', function () {
  $('#camera-feed').toggleClass('rotateimgcw')
  $('#camera-feed').toggleClass('stretch-down')
})
