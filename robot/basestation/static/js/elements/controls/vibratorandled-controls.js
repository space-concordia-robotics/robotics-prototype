$(document).ready(()=> {

    for (let i = 0; i < 2; i++) {
      $('#led' + (i + 1) + '-toggle').click(function (event) {
        event.preventDefault()

        if (!isScienceActivated()) {
          return
        }

        let cmd = 'led' + (i + 1)

        if ($('#led' + (i + 1) + '-toggle').is(':checked')) {
          sendRequest('Science', cmd, function (msgs) {
            console.log('msgs', msgs)

            if (msgs[1].includes(cmd + ' done')) {
              appendToConsole('Success')
            } else {
              appendToConsole('Something went wrong')
            }
          })
        } else {
          cmd += 's'
          sendRequest('Science', cmd, function (msgs) {
            console.log('msgs', msgs)

            if (msgs[1].includes(cmd + ' done')) {
              appendToConsole('Success')
            } else {
              appendToConsole('Something went wrong')
            }
          })
        }
      })
    }

    for (let i = 0; i < 6; i++) {
      $('#vibrator' + (i + 1) + '-toggle').click(function (event) {
        event.preventDefault()

        if (!isScienceActivated()) {
          return
        }

        let cmd = 'v' + (i + 1)

        if ($('#vibrator' + (i + 1) + '-toggle').is(':checked')) {
          sendRequest('Science', cmd, function (msgs) {
            console.log('msgs', msgs)

            if (msgs[1].includes(cmd + ' done')) {
              appendToConsole('Success')
            } else {
              appendToConsole('Something went wrong')
            }
          })
        } else {
          cmd = 'vs'
          sendRequest('Science', cmd, function (msgs) {
            console.log('msgs', msgs)

            if (msgs[1].includes(cmd + ' done')) {
              appendToConsole('Success')
            } else {
              appendToConsole('Something went wrong')
            }
          })
        }
      })
    }
})
