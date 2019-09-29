
$(document).ready(() => {
  $('#turntable-cw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tcw', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#turntable-ccw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tccw', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#turntable-stop-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'ts', function (msgs) {
      console.log('msgs', msgs)
    })
  })
})
